// 三维显示与离屏相机共用同一套CAD网格、关节变换及棋子，不参与棋规判断。
#include "cad_view.hpp"
#include <opencv2/imgproc.hpp>
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <GL/gl.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace {
std::array<Model, 5> arm;
Model boardModel, faceModel;
Texture2D boardTexture;
std::array<Texture2D, 14> faces;
RenderTexture2D cameraTarget, overview;
Camera3D sensor{}, viewer{};
cv::Point2d boardCenter, colAxis, rowAxis;
double boardW, boardH, boardThickness, pieceDiameter, pieceHeight, shaftReference, toolLength,
    screwLead, boardAngle;
double zLow, zHigh, yaw = .7, elevation = .65, distance = 1.1;
bool opened = false;
const std::string names = "kabnrcpKABNRCP";
constexpr double pi = 3.141592653589793;
Vector3 point(cv::Point2d p, double z) { return {float(p.x), float(p.y), float(z)}; }
Texture2D texture(const cv::Mat &bgr) {
    cv::Mat rgba;
    cv::cvtColor(bgr, rgba, bgr.channels() == 4 ? cv::COLOR_BGRA2RGBA : cv::COLOR_BGR2RGBA);
    Image image = {rgba.data, rgba.cols, rgba.rows, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8};
    Texture2D result = LoadTextureFromImage(image);
    SetTextureFilter(result, TEXTURE_FILTER_BILINEAR);
    return result;
}
Model plane(double w, double h) {
    // UV向下对应棋盘行方向；反转三角次序，让顶面法向指向+Z。
    Mesh mesh{};
    mesh.vertexCount = 6;
    mesh.triangleCount = 2;
    mesh.vertices = (float *)MemAlloc(18 * sizeof(float));
    mesh.texcoords = (float *)MemAlloc(12 * sizeof(float));
    const int indices[6] = {0, 2, 1, 0, 3, 2};
    const double uv[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    for (int i = 0; i < 6; ++i) {
        int k = indices[i];
        auto p = colAxis * ((uv[k][0] - .5) * w) + rowAxis * ((uv[k][1] - .5) * h);
        mesh.vertices[3 * i] = p.x;
        mesh.vertices[3 * i + 1] = p.y;
        mesh.vertices[3 * i + 2] = 0;
        mesh.texcoords[2 * i] = uv[k][0];
        mesh.texcoords[2 * i + 1] = uv[k][1];
    }
    UploadMesh(&mesh, false);
    return LoadModelFromMesh(mesh);
}
void world(const std::vector<Piece> &pieces, const std::array<double, 4> &q) {
    double shift = q[0] / 1000 + toolLength - shaftReference;
    if (shift < zLow - 1e-6 || shift > zHigh + 1e-6)
        throw std::runtime_error("三维Z位置超出旧CAD滑台行程");
    rlDisableBackfaceCulling();
    // 网格初始姿态沿+Y，FK肩角从+X计量，因此显示旋转扣除90度。
    for (int g = 0; g < 5; ++g) {
        rlPushMatrix();
        if (g >= 1 && g <= 3)
            rlTranslatef(0, 0, shift);
        if (g == 2)
            rlRotatef(q[1] * RAD2DEG - 90, 0, 0, 1);
        if (g == 3) {
            rlTranslatef(.195 * std::cos(q[1]), .195 * std::sin(q[1]), 0);
            rlRotatef((q[1] + q[2]) * RAD2DEG - 90, 0, 0, 1);
        }
        if (g == 4) {
            rlTranslatef(0, -.075, 0);
            rlRotatef(q[0] / screwLead * 360, 0, 0, 1);
        }
        DrawModel(arm[g], {0, 0, 0}, 1, WHITE);
        rlPopMatrix();
    }
    rlPushMatrix();
    rlTranslatef(boardCenter.x, boardCenter.y, -boardThickness / 2);
    rlRotatef(boardAngle, 0, 0, 1);
    DrawCube({0, 0, 0}, boardW, boardH, boardThickness, {185, 155, 115, 255});
    rlPopMatrix();
    DrawModel(boardModel, point(boardCenter, .0001), 1, WHITE);
    for (const auto &p : pieces) {
        double z = p.held ? q[0] / 1000 : pieceHeight;
        auto xy = p.p * .001;
        Color colour = std::isupper(p.kind) ? Color{145, 35, 35, 255} : Color{35, 35, 40, 255};
        DrawCylinderEx(point(xy, z - pieceHeight), point(xy, z), pieceDiameter / 2,
                       pieceDiameter / 2, 24, colour);
        faceModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = faces[names.find(p.kind)];
        DrawModel(faceModel, point(xy, z + .0002), 1, WHITE);
    }
    cv::Point2d tip((.195 * std::cos(q[1]) + .275 * std::cos(q[1] + q[2])),
                    .195 * std::sin(q[1]) + .275 * std::sin(q[1] + q[2]));
    // 原CAD没有抓手：20mm轴向长度由config标为假设，不能视作实物尺寸。
    DrawCylinderEx(point(tip, q[0] / 1000), point(tip, q[0] / 1000 + toolLength), .006, .006, 24,
                   {70, 75, 85, 255});
    double angle = q[1] + q[2] + q[3];
    DrawLine3D(point(tip, q[0] / 1000 + toolLength),
               point(tip + cv::Point2d(.012 * std::cos(angle), .012 * std::sin(angle)),
                     q[0] / 1000 + toolLength),
               ORANGE);
}
void fit(Texture2D tex, Rectangle area, bool flip = false) {
    if (!tex.id)
        return;
    float scale = std::min(area.width / tex.width, area.height / tex.height);
    Rectangle target = {area.x + (area.width - tex.width * scale) / 2,
                        area.y + (area.height - tex.height * scale) / 2, tex.width * scale,
                        tex.height * scale};
    DrawTexturePro(tex, {0, 0, float(tex.width), float(flip ? -tex.height : tex.height)}, target,
                   {0, 0}, 0, WHITE);
}
void viewCamera() {
    viewer.position =
        Vector3Add(viewer.target, {float(distance * std::cos(elevation) * std::cos(yaw)),
                                   float(distance * std::cos(elevation) * std::sin(yaw)),
                                   float(distance * std::sin(elevation))});
}
void renderOverview(const std::vector<Piece> &pieces, const std::array<double, 4> &joints) {
    BeginTextureMode(overview);
    ClearBackground({225, 230, 235, 255});
    BeginMode3D(viewer);
    for (int i = -8; i <= 8; ++i) {
        DrawLine3D({i * .1f, -.8f, -.014f}, {i * .1f, .8f, -.014f}, {195, 200, 205, 255});
        DrawLine3D({-.8f, i * .1f, -.014f}, {.8f, i * .1f, -.014f}, {195, 200, 205, 255});
    }
    world(pieces, joints);
    EndMode3D();
    EndTextureMode();
}
cv::Mat pixels(Texture2D texture) {
    Image image = LoadImageFromTexture(texture);
    ImageFlipVertical(&image);
    cv::Mat rgba(image.height, image.width, CV_8UC4, image.data), bgr;
    cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
    UnloadImage(image);
    return bgr;
}
} // namespace

void cadOpen(const cv::FileStorage &cfg, const cv::Mat &board,
             const std::array<cv::Mat, 14> &images, bool visible) {
    SetTraceLogLevel(LOG_WARNING);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | (visible ? 0 : FLAG_WINDOW_HIDDEN));
    InitWindow(1000, 820, "SCARA - SolidWorks CAD / OpenCV 5 / YOLO / Pikafish");
    opened = true;
    if (!IsWindowReady())
        throw std::runtime_error("三维OpenGL窗口初始化失败");
    const std::string renderer = reinterpret_cast<const char *>(glGetString(GL_RENDERER));
    if (renderer.find("llvmpipe") != std::string::npos ||
        renderer.find("softpipe") != std::string::npos ||
        renderer.find("Software Rasterizer") != std::string::npos)
        throw std::runtime_error("三维正在CPU软件光栅化，请启用VMware加速3D图形后再运行");
    cv::FileStorage graphics("output/graphics.json",
                             cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    graphics << "renderer" << renderer << "vendor"
             << reinterpret_cast<const char *>(glGetString(GL_VENDOR)) << "version"
             << reinterpret_cast<const char *>(glGetString(GL_VERSION));
    SetTargetFPS(50);
    if (visible)
        SetWindowPosition(40, 70);
    cv::FileStorage manifest("assets/arm/manifest.json", cv::FileStorage::READ);
    if (!manifest.isOpened())
        throw std::runtime_error("缺少SolidWorks网格manifest");
    shaftReference = (double)manifest["shaft_tip_reference_m"];
    zLow = (double)manifest["z_shift_range_m"][0];
    zHigh = (double)manifest["z_shift_range_m"][1];
    toolLength = (double)cfg["tool_length_mm"] / 1000;
    screwLead = (double)cfg["screw_lead_mm"];
    pieceHeight = (double)cfg["piece_height_mm"] / 1000;
    pieceDiameter = (double)cfg["piece_diameter_mm"] / 1000;
    boardAngle = (double)cfg["board_angle_deg"];
    double a = boardAngle * pi / 180;
    colAxis = {std::cos(a), std::sin(a)};
    rowAxis = {std::sin(a), -std::cos(a)};
    boardCenter = {(double)cfg["board_center_mm"][0] / 1000,
                   (double)cfg["board_center_mm"][1] / 1000};
    boardW = (double)cfg["board_size_mm"][0] / 1000;
    boardH = (double)cfg["board_size_mm"][1] / 1000;
    boardThickness = (double)cfg["board_thickness_mm"] / 1000;
    std::ifstream file("assets/arm/arm.mesh", std::ios::binary);
    char magic[8];
    uint32_t count = 0;
    file.read(magic, 8);
    file.read((char *)&count, 4);
    if (!file || std::memcmp(magic, "SCARA3D1", 8) || count != 5)
        throw std::runtime_error("CAD网格格式错误");
    for (auto &model : arm) {
        uint32_t n = 0;
        file.read((char *)&n, 4);
        if (!n || n % 3 || n > 3000000)
            throw std::runtime_error("CAD三角数无效");
        Mesh mesh{};
        mesh.vertexCount = n;
        mesh.triangleCount = n / 3;
        mesh.vertices = (float *)MemAlloc(n * 3 * sizeof(float));
        mesh.colors = (unsigned char *)MemAlloc(n * 4);
        file.read((char *)mesh.vertices, n * 3 * sizeof(float));
        file.read((char *)mesh.colors, n * 4);
        if (!file)
            throw std::runtime_error("CAD网格未读满");
        UploadMesh(&mesh, false);
        model = LoadModelFromMesh(mesh);
    }
    boardTexture = texture(board);
    boardModel = plane(boardW, boardH);
    boardModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = boardTexture;
    faceModel = plane(pieceDiameter, pieceDiameter);
    for (int i = 0; i < 14; ++i)
        faces[i] = texture(images[i]);
    sensor.position = {(float)((double)cfg["camera_position_mm"][0] / 1000),
                       (float)((double)cfg["camera_position_mm"][1] / 1000),
                       (float)((double)cfg["camera_position_mm"][2] / 1000)};
    sensor.target = {sensor.position.x, sensor.position.y, 0};
    sensor.up = {(float)colAxis.x, (float)colAxis.y, 0};
    sensor.fovy = (double)cfg["vertical_fov_deg"];
    sensor.projection = CAMERA_PERSPECTIVE;
    cameraTarget = LoadRenderTexture((int)cfg["image_size_px"][0], (int)cfg["image_size_px"][1]);
    overview = LoadRenderTexture(900, 700);
    viewer.target = {-.10, .18, .20};
    viewer.up = {0, 0, 1};
    viewer.fovy = 45;
    viewer.projection = CAMERA_PERSPECTIVE;
    viewCamera();
}
bool cadValidZ(double zMm) {
    double shift = zMm / 1000 + toolLength - shaftReference;
    return shift >= zLow && shift <= zHigh;
}
cv::Mat cadCamera(const std::vector<Piece> &pieces, const std::array<double, 4> &joints) {
    BeginTextureMode(cameraTarget);
    ClearBackground({88, 96, 92, 255});
    BeginMode3D(sensor);
    world(pieces, joints);
    EndMode3D();
    EndTextureMode();
    return pixels(cameraTarget.texture);
}
cv::Mat cadScene(const std::vector<Piece> &pieces, const std::array<double, 4> &joints) {
    renderOverview(pieces, joints);
    return pixels(overview.texture);
}
CadInput cadShow(const std::vector<Piece> &pieces, const std::array<double, 4> &joints,
                 const std::string &caption, const std::string &message, int dragging,
                 cv::Point2d ghost) {
    float scale = std::min(GetScreenWidth() / 1000.f, GetScreenHeight() / 820.f);
    Vector2 offset = {(GetScreenWidth() - 1000 * scale) / 2, (GetScreenHeight() - 820 * scale) / 2};
    Vector2 mouse = GetMousePosition();
    mouse = {(mouse.x - offset.x) / scale, (mouse.y - offset.y) / scale};
    Rectangle area = {50, 65, 900, 700};
    bool inside = CheckCollisionPointRec(mouse, area);
    if (inside && IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        auto d = GetMouseDelta();
        yaw -= d.x * .008;
        elevation = std::clamp(elevation + d.y * .005, .55, 1.3);
    }
    if (inside)
        distance = std::clamp(distance * std::pow(.9, GetMouseWheelMove()), .65, 2.5);
    if (IsKeyPressed(KEY_R)) {
        yaw = .7;
        elevation = .65;
        distance = 1.1;
    }
    viewCamera();
    renderOverview(pieces, joints);
    if (dragging >= 0) {
        BeginTextureMode(overview);
        BeginMode3D(viewer);
        DrawSphereWires(point(ghost * .001, pieceHeight + .004), pieceDiameter / 2, 8, 16, ORANGE);
        EndMode3D();
        EndTextureMode();
    }
    BeginDrawing();
    ClearBackground({25, 30, 36, 255});
    Camera2D screen{};
    screen.offset = offset;
    screen.zoom = scale;
    BeginMode2D(screen);
    DrawText(caption.c_str(), 20, 12, 20, RAYWHITE);
    DrawText("SolidWorks mesh | RMB orbit | Wheel zoom | R reset view", 20, 40, 16, RAYWHITE);
    fit(overview.texture, area, true);
    DrawText(message.c_str(), 20, 772, 16, RAYWHITE);
    DrawText(
        TextFormat("CAD FPS %d | SPACE pause | N step | ESC exit | Tool: simplified", GetFPS()), 20,
        798, 16, RAYWHITE);
    EndMode2D();
    CadInput input;
    input.pause = IsKeyPressed(KEY_SPACE);
    input.step = IsKeyPressed(KEY_N);
    input.quit = WindowShouldClose();
    input.mouse = IsMouseButtonPressed(MOUSE_BUTTON_LEFT)    ? 1
                  : IsMouseButtonReleased(MOUSE_BUTTON_LEFT) ? 3
                  : IsMouseButtonDown(MOUSE_BUTTON_LEFT)     ? 2
                                                             : 0;
    if (inside) {
        Ray ray = GetScreenToWorldRayEx({mouse.x - area.x, mouse.y - area.y}, viewer, 900, 700);
        if (std::abs(ray.direction.z) > 1e-6) {
            float t = (pieceHeight - ray.position.z) / ray.direction.z;
            if (t > 0) {
                input.overBoard = true;
                input.world = {(ray.position.x + t * ray.direction.x) * 1000,
                               (ray.position.y + t * ray.direction.y) * 1000};
            }
        }
    }
    EndDrawing();
    return input;
}
void cadClose() {
    if (!opened)
        return;
    for (auto m : arm)
        if (m.meshCount)
            UnloadModel(m);
    if (boardModel.meshCount)
        UnloadModel(boardModel);
    if (faceModel.meshCount)
        UnloadModel(faceModel);
    if (boardTexture.id)
        UnloadTexture(boardTexture);
    for (auto t : faces)
        if (t.id)
            UnloadTexture(t);
    if (cameraTarget.id)
        UnloadRenderTexture(cameraTarget);
    if (overview.id)
        UnloadRenderTexture(overview);
    CloseWindow();
    opened = false;
}

#include "chess/board.hpp"
#include "chess/pikafish.hpp"
#include "vision/board_vision.hpp"
#include "vision/camera.hpp"
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <cmath>
#include <limits>

namespace {
void check(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}

chess::Observation observation(const chess::Board& board)
{
    return {board, true, true, false};
}

chess::Board moved(const chess::Board& before, bool red, const std::string& text)
{
    chess::Move move;
    chess::Board after;
    std::string error;
    check(chess::parseMove(text, move), "test move parse");
    check(chess::applyMove(before, red, move, after, error), "test move application");
    return after;
}

void terminalTests()
{
    // 两局已由4762ca3原始源码的真实Pikafish核验perft=0：前者Checkers=e8，后者为空。
    // 环境变量可指定独立本地引擎再次查询；不设置时只测试棋局对已知空合法表的处理。
    const char* positions[][3] = {
        {"3nkn3/R6R1/9/9/4P4/9/9/9/9/4K4 w - - 0 1", "h8e8",
         "3nkn3/R3R4/9/9/4P4/9/9/9/9/4K4 b - - 0 1"},  // 将死
        {"4k4/3R1r3/5R3/9/4P4/9/9/9/9/4K4 w - - 0 1", "f7f8",
         "4k4/3R1R3/9/9/4P4/9/9/9/9/4K4 b - - 0 1"}    // 吃车后困毙
    };
    const char* directory = std::getenv("SCARA_TEST_PIKAFISH_DIR");
    Pikafish engine;
    if (directory) check(engine.start(directory), "start independent terminal-test engine");
    for (const auto& position : positions) {
        chess::Board board, before;
        bool red = true;
        std::string error;
        check(chess::parseFen(position[0], before, red, error) && red, "terminal predecessor FEN");
        check(chess::parseFen(position[2], board, red, error) && !red, "terminal FEN with black to move");
        check(board[0][4] == 'k' && board[9][4] == 'K', "terminal state retains both generals");
        std::vector<std::string> legal{position[1]};
        if (directory) check(engine.legalMoves(position[0], "", legal), "query actual predecessor legal moves");
        chess::Game game;
        chess::Move move;
        check(game.reset(before) && game.acceptHuman(observation(board), legal, move), "terminal move is legal and observed");
        check(game.baseFen() == position[0] && game.history() == position[1], "keep baseline and terminal move history");
        legal.clear();
        if (directory) check(engine.legalMoves(game.baseFen(), game.history(), legal), "query terminal through complete history");
        check(legal.empty(), "mate and stalemate both have zero legal moves");
        check(game.judge(legal) && game.result() == chess::Result::RedWon,
              "black loses for both mate and stalemate");
    }
    check(engine.close(), "terminal-test engine is reaped");
}

void protocolTests()
{
    // 只启动本测试的本地协议替身，不连接正在运行的板端引擎。
    char pattern[] = "/tmp/scara-chess-XXXXXX";
    char* name = mkdtemp(pattern);
    check(name != nullptr, "temporary engine directory");
    struct Directory {
        std::filesystem::path path;
        ~Directory() { std::error_code error; std::filesystem::remove_all(path, error); }
    } directory{name};
    const auto executable = directory.path / "pikafish";
    {
        std::ofstream script(executable);
        script << "#!/bin/sh\n"
                  "while IFS= read -r line; do\n"
                  "case \"$line\" in\n"
                  "uci) printf 'uciok\\n' ;;\n"
                  "isready) printf 'readyok\\n' ;;\n"
                  "position*) printf '%s\\n' \"$line\" > seen-position ;;\n"
                  "'go perft 1') printf 'e3e4: 1\\nNodes searched: 1\\n' ;;\n"
                  "'go movetime '*) printf 'info depth 1\\nbestmove e3e4\\n' ;;\n"
                  "quit) exit 0 ;;\n"
                  "esac\ndone\n";
    }
    std::filesystem::permissions(executable, std::filesystem::perms::owner_all);
    Pikafish engine;
    check(engine.start(name), "fake engine handshake");
    std::vector<std::string> legal;
    check(engine.legalMoves("", legal) && legal == std::vector<std::string>{"e3e4"}, "legacy perft interface");
    std::string seen;
    { std::ifstream file(directory.path / "seen-position"); std::getline(file, seen); }
    check(seen == "position startpos", "legacy startpos command");
    auto board = chess::initialBoard();
    board[9][0] = '.';
    const std::string base = chess::fen(board, true);
    check(engine.legalMoves(base, "c3c4 c6c5", legal), "FEN and history perft");
    { std::ifstream file(directory.path / "seen-position"); std::getline(file, seen); }
    check(seen == "position fen " + base + " moves c3c4 c6c5", "preserve custom FEN plus full history");
    std::string best;
    check(engine.think(base, "c3c4 c6c5", 1, best) && best == "e3e4", "FEN search interface");
    check(!engine.legalMoves(base + "\nquit", "", legal), "reject FEN command injection");
    check(engine.start(name), "restart local fake engine");
    check(!engine.legalMoves("a0a0", legal), "reject zero-length move in history");
}
} // namespace

void testChessRules()
{
    using namespace chess;
    const Board start = initialBoard();
    const std::string startFen = "rnbakabnr/9/1c5c1/p1p1p1p1p/9/9/P1P1P1P1P/1C5C1/9/RNBAKABNR w - - 0 1";
    check(fen(start, true) == startFen, "initial FEN and board direction");
    Move move;
    check(parseMove("a0i9", move) && move.fromRow == 9 && move.fromCol == 0 &&
          move.toRow == 0 && move.toCol == 8 && uci(move) == "a0i9", "UCI rank direction");
    check(!parseMove("a0a0", move) && !parseMove("j0a0", move) &&
          !parseMove("a0i10", move) && uci(Move{}).empty(), "reject invalid coordinates");
    Board parsed;
    bool red = false;
    std::string error;
    check(parseFen(startFen, parsed, red, error) && parsed == start && red, "FEN roundtrip");
    check(!parseFen("9/9 w - - 0 1", parsed, red, error), "reject incomplete FEN");
    check(!parseFen(startFen + " extra", parsed, red, error), "reject extra FEN fields");
    Board unknown = start; unknown[4][0] = '?';
    check(!validBoard(unknown, error) && fen(unknown, true).empty(), "unknown is not empty");
    Board missingKing = start; missingKing[0][4] = '.';
    check(!validBoard(missingKing, error), "missing general is a vision/setup error");
    Board tooMany = start; tooMany[4][0] = 'R';
    check(!validBoard(tooMany, error), "reject impossible piece inventory");

    const Board afterHuman = moved(start, true, "e3e4");
    auto seen = observation(afterHuman);
    Game game;
    seen.stable = false;
    check(!game.acceptHuman(seen, {"e3e4"}, move) && game.history().empty(), "unstable frame not committed");
    seen = observation(afterHuman); seen.complete = false;
    check(!game.acceptHuman(seen, {"e3e4"}, move), "incomplete frame rejected");
    seen = observation(afterHuman); seen.duplicate = true;
    check(!game.acceptHuman(seen, {"e3e4"}, move), "duplicate detection rejected");
    check(!game.acceptHuman(observation(unknown), {"e3e4"}, move), "unknown frame rejected");
    check(!game.acceptHuman(observation(missingKing), {"e3e4"}, move) &&
          game.result() == Result::Playing, "missing general must not end game");
    check(!inferMove(start, observation(afterHuman), true, {"e3e4", "e3e4"}, move, error), "duplicate legal list rejected");
    check(!inferMove(start, observation(start), true, {"e3e4"}, move, error), "unchanged image is not a move");
    check(!game.acceptHuman(observation(afterHuman), {"c3c4"}, move), "reject nonlegal board transition");
    check(game.acceptHuman(observation(afterHuman), {"c3c4", "e3e4"}, move) &&
          uci(move) == "e3e4" && !game.redToMove(), "accept unique legal human move");
    check(!game.prepareRobot(observation(start), "a6a5", {"a6a5"}, move), "pre-motion board must match");
    check(!game.prepareRobot(observation(afterHuman), "c6c5", {"a6a5"}, move), "reject unlisted engine choice");
    check(game.prepareRobot(observation(afterHuman), "a6a5", {"a6a5"}, move) &&
          game.pending() && game.board() == afterHuman && game.history() == "e3e4", "plan does not commit motion");
    const Board afterRobot = moved(afterHuman, false, "a6a5");
    check(!game.judge({}), "no terminal judgment during pending motion");
    check(!game.confirmRobot(observation(afterRobot), false), "failed actuator cannot commit");
    check(!game.confirmRobot(observation(afterHuman), true), "wrong after-image cannot commit");
    check(game.pending() && game.history() == "e3e4", "failed confirmation preserves confirmed history");
    check(game.confirmRobot(observation(afterRobot), true) && !game.pending() &&
          game.redToMove() && game.history() == "e3e4 a6a5", "commit only after arm and vision confirmation");
    check(!game.confirmRobot(observation(afterRobot), true), "duplicate completion cannot commit twice");

    Board capture = start;
    capture[6][0] = '.'; capture[4][0] = 'P';
    Board captured = moved(capture, true, "a5a6");
    check(inferMove(capture, observation(captured), true, {"a5a6", "c3c4"}, move, error) &&
          uci(move) == "a5a6" && captured[3][0] == 'P' && captured[4][0] == '.', "infer capture by full board difference");
    check(game.reset(capture) && game.baseFen() == fen(capture, true) && game.history().empty(), "custom confirmed baseline");
    check(game.judge({}) && game.result() == Result::BlackWon, "red with no legal move loses even with king present");
    check(game.reset(start, false) && game.judge({}) && game.result() == Result::RedWon, "black stalemate loses");
    check(game.reset(start) && game.agreeDraw() && game.result() == Result::Draw, "agreed draw");
    check(game.reset(start) && game.resign(false) && game.result() == Result::RedWon, "explicit resignation");

    // 合法马往返让相同盘面和轮次出现三次；只暂停，不擅自宣布和棋。
    check(game.reset(start), "reset repetition game");
    const std::string cycle[] = {"b0c2", "b9c7", "c2b0", "c7b9"};
    for (int i = 0; i < 8; ++i) {
        const std::string next = cycle[i % 4];
        const Board after = moved(game.board(), game.redToMove(), next);
        if (game.redToMove()) check(game.acceptHuman(observation(after), {next}, move), "repetition human move");
        else {
            check(game.prepareRobot(observation(game.board()), next, {next}, move), "repetition robot plan");
            check(game.confirmRobot(observation(after), true), "repetition robot confirmation");
        }
        check(game.judge({cycle[(i + 1) % 4]}), "repetition judgment");
    }
    check(game.result() == Result::ReviewRequired, "third repetition requires review, not automatic draw");
    check(game.agreeDraw() && game.result() == Result::Draw, "participants may agree draw after review pause");
    terminalTests();
    protocolTests();
}

namespace {
VisionConfig exampleVision()
{
    // 明确的合成图参数，只验证几何和接口，不是实物尺寸或识别验收。
    VisionConfig c;
    c.boardWidthMm = 480; c.boardHeightMm = 520;
    c.gridXmm = 80; c.gridYmm = 80; c.pitchMm = 40; c.markerSideMm = 48;
    c.markerCentersMm = {{{32,32}, {448,32}, {448,488}, {32,488}}};
    c.tcpMarkerId = 7; c.tcpCalibrated = true;
    c.tcpPlaneZmm = 12; c.tcpPlaneToleranceMm = .2;
    c.maxObservationAgeSeconds = .5;
    return c;
}

void paintMarker(cv::Mat& image, int id, cv::Point center, bool turned = false)
{
    cv::Mat marker, colored;
    cv::aruco::generateImageMarker(
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), id, 48, marker);
    if (turned) cv::rotate(marker, marker, cv::ROTATE_90_CLOCKWISE);
    cv::cvtColor(marker, colored, cv::COLOR_GRAY2BGR);
    image(cv::Rect(center.x-32,center.y-32,64,64)).setTo(cv::Scalar(255,255,255));
    colored.copyTo(image(cv::Rect(center.x-24,center.y-24,48,48)));
}

cv::Mat cameraBoard(int missing = -1, int turned = -1)
{
    const auto c = exampleVision();
    cv::Mat image(960,1280,CV_8UC3,cv::Scalar(40,40,40));
    image(cv::Rect(200,100,480,520)).setTo(cv::Scalar(106,167,199));
    for (int id = 0; id < 4; ++id) {
        if (id == missing) continue;
        const auto p = c.markerCentersMm[id];
        paintMarker(image,id,{int(p.x)+200,int(p.y)+100},id==turned);
    }
    return image;
}

std::vector<Detection> initialDetections()
{
    const auto board = chess::initialBoard();
    const std::string classes = "kabnrcpKABNRCP";
    std::vector<Detection> pieces;
    for (int row = 0; row < 10; ++row) {
        for (int col = 0; col < 9; ++col) {
            if (board[row][col] == '.') continue;
            const float x = float(40 + 80 * col), y = float(40 + 80 * row);
            pieces.push_back({int(classes.find(board[row][col])),.95f,x-25,y-25,x+25,y+25});
        }
    }
    return pieces;
}

void checkStable(BoardVision& vision, const cv::Mat& camera, const std::vector<Detection>& pieces)
{
    vision.reset();
    cv::Mat rgb;
    std::string error;
    for (int i = 0; i < 3; ++i) {
        check(vision.locate(camera,rgb,error), "four marker location");
        check(rgb.size() == cv::Size(720,800), "configured crop size");
        const auto color = rgb.at<cv::Vec3b>(80,80);
        check(color[0] == 199 && color[1] == 167 && color[2] == 106, "BGR converts to RGB");
        auto found = vision.interpret(pieces);
        check(found.complete && !found.duplicate && found.board == chess::initialBoard(), "90-square map");
        check(found.stable == (i == 2), "distinct complete frames required");
    }
}

double steadySeconds()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void visionTests()
{
    check(cv::getVersionString() == "5.0.0", "OpenCV exactly 5.0.0");
    cv::setNumThreads(1);
    auto config = exampleVision();
    BoardVision vision(config);
    const auto camera = cameraBoard();
    const auto pieces = initialDetections();
    checkStable(vision,camera,pieces);
    check(!vision.interpret(pieces).complete, "same frame cannot be counted again");
    cv::Mat rgb;
    std::string error;
    check(!vision.locate(cameraBoard(2),rgb,error) && rgb.empty() && vision.boardImage().empty(),
          "missing marker clears old geometry");
    check(!vision.interpret(pieces).complete, "lost geometry cannot interpret old detections");
    cv::Mat duplicate = camera.clone();
    paintMarker(duplicate,0,{1050,150});
    check(!vision.locate(duplicate,rgb,error), "duplicate corner ID rejected");
    check(!vision.locate(cameraBoard(-1,0),rgb,error), "one rotated marker rejected");
    cv::Mat rotated;
    cv::rotate(camera,rotated,cv::ROTATE_90_CLOCKWISE);
    checkStable(vision,rotated,pieces);
    checkStable(vision,camera,pieces);
    auto bad = pieces;
    bad.front().confidence = .49f;
    check(vision.locate(camera,rgb,error), "fresh image before low confidence");
    auto found = vision.interpret(bad);
    check(!found.complete && found.board[0][0] == '?', "unknown is not an empty intersection");
    bad = pieces; bad.push_back(pieces.front());
    check(vision.locate(camera,rgb,error), "fresh image before duplicate piece");
    found = vision.interpret(bad);
    check(!found.complete && found.duplicate, "duplicate piece rejected");
    bad = pieces; bad.front().x1 += 35; bad.front().x2 += 35;
    check(vision.locate(camera,rgb,error), "fresh image before off-grid piece");
    check(!vision.interpret(bad).complete, "off-grid center rejected");
    bad = pieces; bad.front().classId = 14;
    check(vision.locate(camera,rgb,error) && !vision.interpret(bad).complete, "invalid class rejected");
    bad = pieces; bad.front().confidence = std::numeric_limits<float>::quiet_NaN();
    check(vision.locate(camera,rgb,error) && !vision.interpret(bad).complete, "NaN confidence rejected");
    bad = pieces; bad.front().x1 = bad.front().x2;
    check(vision.locate(camera,rgb,error) && !vision.interpret(bad).complete, "degenerate box rejected");
    check(!vision.locate(cv::Mat(),rgb,error), "empty image rejected");
    bool rejected = false;
    try { vision.squareXY(0,0); } catch (const std::runtime_error&) { rejected = true; }
    check(rejected, "uncalibrated board cannot produce mechanical targets");
    config.boardCalibrated = true;
    config.boardToBase = {0,-1,260, 1,0,10, 0,0,1};
    for (int i = 0; i < 4; ++i) config.referenceMarkerPixels[i] = config.markerCentersMm[i] + cv::Point2f(200,100);
    BoardVision calibrated(config);
    check(calibrated.locate(camera,rgb,error), "calibrated reference accepted");
    check(cv::norm(calibrated.squareXY(0,0) - cv::Point2d(180,90)) < 1e-9, "board-to-base mapping");
    cv::Mat shifted;
    cv::warpAffine(camera,shifted,cv::Mat(cv::Matx23d(1,0,20, 0,1,0)),camera.size());
    check(!calibrated.locate(shifted,rgb,error), "moved board cannot reuse fixed base calibration");
    cv::Mat tcpFrame = camera.clone();
    paintMarker(tcpFrame,7,{900,700});
    auto tcp = calibrated.observeTcp(tcpFrame,steadySeconds(),12);
    check(tcp.valid && cv::norm(tcp.xy-cv::Point2d(899.5,699.5)) < 1.0, "independent fresh marker measurement");
    check(!calibrated.observeTcp(tcpFrame,steadySeconds()-2,12).valid, "old TCP observation rejected");
    check(!calibrated.observeTcp(tcpFrame,steadySeconds()+2,12).valid, "future TCP observation rejected");
    check(!calibrated.observeTcp(tcpFrame,steadySeconds(),20).valid, "other Z plane rejected");
    check(!calibrated.observeTcp(camera,steadySeconds(),12).valid, "no marker is not synthetic XY");
    paintMarker(tcpFrame,7,{1100,700});
    check(!calibrated.observeTcp(tcpFrame,steadySeconds(),12).valid, "duplicate TCP markers rejected");
    rejected = false;
    try { Camera invalid(1279,960); } catch (const std::invalid_argument&) { rejected = true; }
    check(rejected, "NV12 odd width rejected without opening hardware");
}

void restoreAndColourTests()
{
    const auto start = chess::initialBoard();
    const auto afterRed = moved(start,true,"e3e4");
    const auto afterBlack = moved(afterRed,false,"a6a5");
    chess::Game game;
    chess::Move move;
    check(game.reset(start,true,false), "robot red with human black");
    check(!game.acceptHuman(observation(afterRed),{"e3e4"},move), "human cannot move robot red");
    check(game.prepareRobot(observation(start),"e3e4",{"e3e4"},move), "robot red moves first");
    check(!game.restore(chess::fen(start,true),"",false), "pending motion blocks restore");
    check(game.confirmRobot(observation(afterRed),true), "robot red confirmation");
    check(game.acceptHuman(observation(afterBlack),{"a6a5"},move), "human black follows");
    const auto history = game.history();
    chess::Game restored;
    check(restored.restore(chess::fen(start,true),history,false), "restore full confirmed history");
    check(restored.board() == game.board() && restored.redToMove() == game.redToMove() &&
          restored.history() == history && restored.baseFen() == game.baseFen(), "restore preserves baseline and turn");
    check(!restored.restore(chess::fen(start,true),"e3e4  a6a5",false), "reject malformed journal history");
    check(restored.history() == history && restored.board() == afterBlack, "failed restore leaves original state");
    check(!restored.restore(chess::fen(start,true),"a4a5",false), "empty-source history rejected");
    const std::string repeated = "b0c2 b9c7 c2b0 c7b9 b0c2 b9c7 c2b0 c7b9";
    check(restored.restore(chess::fen(start,true),repeated,true) &&
          restored.result() == chess::Result::ReviewRequired, "restored repetition remains review required");
}
}

void testVisionChess()
{
    testChessRules();
    visionTests();
    restoreAndColourTests();
}

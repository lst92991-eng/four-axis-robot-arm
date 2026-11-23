// 检查离线布局，将格点数值与原图叠图作为同一份标定的两个输出。
#include "bench/legacy_site.hpp"
#include "vision/geometry.hpp"
#include <stdexcept>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <iostream>

//---------固定棋盘坐标导出区---------
int main(int argc, char **argv)
{
    try
    {
        if (argc != 4 || std::string(argv[1]) != "inspect")
            throw std::runtime_error("用法：scara_geometry_tool inspect layout.json grid.json");
        //与正式程序共用同一个初始化入口，参数与90点坐标只生成一次
        cv::FileStorage config, report;
        scara::Geometry geometry;
        if (!config.open(argv[2], cv::FileStorage::READ)) throw std::runtime_error("无法打开布局");
        cv::Matx23d pixelToBase;
        scara::Bench_geometry(config.root(), geometry, &pixelToBase);
        // 从布局文件所在目录找到当时的相机原图，用同一坐标系绘制交点。
        std::filesystem::path path = std::filesystem::u8path(argv[2]).parent_path();
        path /= std::string(config["image"]);
        cv::Mat source = cv::imread(path.string()), overlay;
        if (source.size() != cv::Size(scara::ImageWidth, scara::ImageHeight))
            throw std::runtime_error("需要与布局一致的1280×720原图");
        // 报告同时保留标称格点和实际取放目标，方便核对单格毫米修正。
        if (!report.open(argv[3], cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON))
            throw std::runtime_error("无法创建格点报告");
        report << "schema_version" << 4 << "row_origin" << "black" << "coordinate_order" << "row,column" << "mapping_model" << "affine";
        report << "frame_id" << std::string(config["frame_id"]);
        report << "geometry_valid" << 1 << "orientation_valid" << 1;
        report << "pixel_to_base" << cv::Mat(pixelToBase);
        report << "points" << "[";
        for (int i = 0; i < scara::Cells; ++i)
        {
            int row = i / scara::Columns, column = i % scara::Columns;
            // pixel 用于原图叠加，base 是按三点步长算出的标称位置。
            cv::Point2d pixel = geometry.cell[row][column].pixel;
            cv::Point2d base = geometry.origin.xy + row * geometry.rowStep.xy + column * geometry.columnStep.xy;
            report << "{" << "column" << column << "row" << row;
            report << "pixel_geometric" << "[" << pixel.x << pixel.y << "]";
            // target 来自标定表，包含为这个交点配置的取放修正。
            cv::Point2d target = geometry.cell[row][column].xy;
            report << "base_xy_valid" << 1 << "base_xy_mm" << "[" << base.x << base.y << "]";
            report << "target_xy_mm" << "[" << target.x << target.y << "]" << "}";
        }
        report << "]";
        report.release();
        // 在原图上画出与报告对应的 90 个交点，按报告名加 .png 保存。
        scara::Geometry_draw(source, geometry, overlay);
        if (!cv::imwrite(std::string(argv[3]) + ".png", overlay)) throw std::runtime_error("无法保存叠图");
        std::cout << "三点对应关系及90点叠图已生成\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

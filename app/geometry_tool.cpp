//离线读取一份布局，导出同源的90点坐标、六个系数和原图叠图
#include "vision/geometry.hpp"
#include "vision/io.hpp"
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <iostream>

int main(int argc, char **argv)
{
    try
    {
        if (argc != 4 || std::string(argv[1]) != "inspect")
            throw std::runtime_error("用法：scara_geometry_tool inspect layout.json grid.json");
        //与正式程序共用同一个初始化入口，参数与90点坐标只生成一次
        cv::FileStorage config, report;
        scara::Geometry geometry;
        scara::readStorage(argv[2], config);
        scara::Geometry_init(config.root(), geometry);
        std::filesystem::path path = std::filesystem::u8path(argv[2]).parent_path();
        path /= std::string(config["image"]);
        cv::Mat source = cv::imread(path.string()), overlay;
        if (source.size() != cv::Size(scara::ImageWidth, scara::ImageHeight))
            throw std::runtime_error("需要与布局一致的1280×720原图");
        //报告只保存现有页面和取放脚本需要的数据，六个系数与格点来自同一次初始化
        scara::memoryStorage(true, report);
        report << "schema_version" << 3 << "mapping_model" << "affine";
        report << "frame_id" << std::string(config["frame_id"]);
        report << "geometry_valid" << 1 << "orientation_valid" << 1;
        report << "pixel_to_base" << cv::Mat(geometry.pixelToBase);
        report << "points" << "[";
        for (int i = 0; i < scara::Cells; ++i)
        {
            cv::Point2d pixel = geometry.pixels[i], base = geometry.targets[i];
            report << "{" << "column" << i % scara::Columns << "row" << i / scara::Columns;
            report << "pixel_geometric" << "[" << pixel.x << pixel.y << "]";
            report << "base_xy_valid" << 1 << "base_xy_mm" << "[" << base.x << base.y << "]" << "}";
        }
        report << "]";
        scara::writeBytes(argv[3], report.releaseAndGetString());
        //原图叠图供检查交点和显示方向，识别ROI保持原有设置
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

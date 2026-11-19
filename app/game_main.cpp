//程序入口：读取配置，组织双方回合，处理退出收尾
#include "game_files.hpp"
#include "vision/capture.hpp"
#include <iostream>
#include <stdexcept>

//对局运行标志，供主程序和停止信号回调访问
volatile std::sig_atomic_t scara::gameRunning = 1;

//---------停止回调区---------
static void requestStop(int)
{
    //通知主循环、引擎搜索和机械执行结束当前任务
    scara::gameRunning = 0;
    //请求取流停止，让等待图像的代码能够退出
    scara::Camera_stop();
}

//---------主程序区---------
int main(int argc, char **argv)
{
    using namespace scara;
    //启动参数为配置路径和可选模型路径，数量不符时返回2
    if (argc < 2 || argc > 3) return 2;
    //棋盘布局、机械参考、示教高度和对局设置的文件路径
    const char *configPath = argv[1];
    //实机使用的RKNN模型路径，仿真运行时可省略
    const char *modelPath = argc == 3 ? argv[2] : "";
    //启动时载入的相机、引擎、观察位和对局参数
    GameConfig config;
    //本局棋盘、识别结果、机械参考和待执行动作
    Session session;
    //进程退出码，0表示正常结束，1表示运行或收尾失败
    int result = 0;
    //将Ctrl+C与进程终止信号交给同一个停止回调
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    //统一注册屏幕显示回调，具体原因由各处理步骤提供
    for (size_t i = 0; i < size_t(EventType::Count); ++i)
        Event_on(static_cast<EventType>(i), Files_event);
    try
    {
        //读取配置与标定，建立棋盘坐标和机械参考
        Match_load(configPath, config, session);
        //初始化运行资源和输出目录，实机模式准备字图与引擎取消条件
        Runtime_open(config, modelPath);
        //机械臂进入观察位，由操作员确认识别到的开局盘面
        Match_start(config, session);

        //每轮处理一个回合，棋局状态决定下一轮由谁走棋
        while (gameRunning)
        {
            //本回合合法走法，用于核对人类走法和机器人应手
            std::vector<std::string> legal;
            //机器人回合由引擎选出的走法，例如a3a4
            std::string choice;

            //发布当前棋局和思考状态
            Match_publishStatus("thinking", session);
            //按起始局面与完整走棋历史查询合法走法和机器人应手
            try { Engine_query(session.game, legal, choice); }
            catch (const std::exception &error)
            {
                if (!gameRunning) throw;
                Event_emit(EventType::Engine, error.what());
                Match_publishStatus("engine_recovery", session);
                Command command;
                do
                {
                    Operator_read("engine_recovery", "处理引擎问题后回车继续当前棋局；quit退出。", command);
                } while (command != Command::Ready);
                continue;
            }
            //判断终局与重复局面，状态异常时转入统一收尾
            if (!chess::Game_judge(session.game, legal)) throw std::runtime_error(session.game.error);
            //棋局仍在进行时，处理当前一方的回合
            if (session.game.result == chess::Result::Playing)
            {
                if (session.game.redToMove == session.game.humanRed)
                    //玩家可在实体棋盘走棋，也可选择让机械臂代走
                    Match_human(config, session, legal);
                else
                    //执行机器人取放，用新画面确认落子后更新棋局
                    Match_executeMove(config, session, legal, choice);
            }
            //出现胜负、和棋或待人工裁定时结束主循环
            if (session.game.result != chess::Result::Playing) gameRunning = 0;
        }
    }
    //初始化和回合处理的异常在此记录，随后统一收尾
    catch (const std::exception &error)
    {
        //停止请求按正常退出处理，运行异常使用失败退出码
        result = gameRunning ? 1 : 0;
        //运行异常写入棋局状态，供页面显示
        if (gameRunning) session.game.error = error.what();
        if (gameRunning) Event_emit(EventType::Application, error.what());
    }

    //请求机械停止，并关闭引擎进程、相机和NPU
    if (!Runtime_close())
    {
        //停止或资源回收未确认时，记录失败原因
        result = 1;
        session.game.error += "；机械停止或资源回收未确认";
    }
    //根据棋局结果生成最终显示状态
    std::string phase = "finished";
    //棋局仍在进行时退出，记录为中途停止
    if (session.game.result == chess::Result::Playing) phase = "stopped";
    //重复局面等情况交由人工裁定
    if (session.game.result == chess::Result::ReviewRequired) phase = "review_required";
    //运行或收尾失败时，最终状态记录为错误
    if (result)
    {
        phase = "error";
        //终端输出最终错误，包含运行或收尾阶段的失败原因
        std::cerr << session.game.error << '\n';
    }
    //发布最终棋局和结束原因
    Match_publishStatus(phase, session);
    return result;
}

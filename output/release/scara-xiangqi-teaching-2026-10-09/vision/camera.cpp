#include "camera.hpp"
#include "app/config.hpp"
#include <opencv2/imgproc.hpp>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <fcntl.h>
#include <poll.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace {
const char* work="/tmp/scara-robot-camera";
const char* fifo="/tmp/scara-robot-camera/frame.nv12";
const char* lock="/tmp/scara-vision-capture.lock";

// 只回收自己启动的进程，先TERM，再有限等待，最后KILL。
bool finish(int& pid)
{
    if (pid<=0) return true;
    int status=0;
    for (int signal : {SIGTERM, SIGKILL}) {
        int result=waitpid(pid,&status,WNOHANG);
        if (result==pid || (result<0 && errno==ECHILD)) { pid=-1; return true; }
        if (result<0 && errno!=EINTR) return false;
        kill(pid,signal);
        const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
        while (std::chrono::steady_clock::now()<deadline) {
            result=waitpid(pid,&status,WNOHANG);
            if (result==pid || (result<0 && errno==ECHILD)) { pid=-1; return true; }
            if (result<0 && errno!=EINTR) return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
    }
    return false;  // 包括KILL后的等待也有界，失败不能伪称进程已经消失。
}

void redirectLog()
{
    int log=open("/tmp/scara-robot-camera/camera.log",O_WRONLY|O_CREAT|O_APPEND,0600);
    if (log>=0) { dup2(log,STDOUT_FILENO); dup2(log,STDERR_FILENO); close(log); }
}
}

bool Camera::start(const std::string& script, std::string& error)
{
    error.clear();
    if (!stop()) { error="上次相机进程尚未回收"; return false; }
    if (cancelled && cancelled()) { error="用户取消相机启动"; return false; }
    if (mkdir(work,0700)<0 && errno!=EEXIST) { error="无法建立相机工作目录"; return false; }
    if (mkdir(lock,0700)<0) { error="摄像头正在被其他预览占用"; return false; }
    locked_=true;
    loaderPid_=fork();
    if (loaderPid_==0) { redirectLog(); execl("/bin/sh","sh",script.c_str(),nullptr); _exit(127); }
    if (loaderPid_<0) { error="无法启动相机模块脚本"; stop(); return false; }
    int status=0;
    bool loaded=false, wasCancelled=false;
    for (int i=0;i<200;++i) {
        if (cancelled && cancelled()) { wasCancelled=true; break; }
        int result=waitpid(loaderPid_,&status,WNOHANG);
        if (result==loaderPid_) { loaded=WIFEXITED(status) && WEXITSTATUS(status)==0; loaderPid_=-1; break; }
        if (result<0 && errno!=EINTR) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    if (!loaded) {
        error=wasCancelled ? "相机模块加载已取消" : "相机模块加载失败，查看camera.log";
        stop();
        return false;
    }
    unlink(fifo);
    if (mkfifo(fifo,0600)<0) { error="无法建立帧管道"; stop(); return false; }
    // RDWR保持FIFO在ISP初始化期间也打开；实际数据只从fd_读取。
    fd_=open(fifo,O_RDWR|O_NONBLOCK|O_CLOEXEC);
    if (fd_<0) { error="无法打开帧管道"; stop(); return false; }
    pid_=fork();
    if (pid_==0) {
        redirectLog();
        execlp("rkisp_demo","rkisp_demo","--device","/dev/video11",
               "--width","1280","--height","960","--format","NV12",
               "--count","1000030","--stream-skip","30","--stream-count","1000000",
               "--stream-to",fifo,"--rkaiq","--iqpath","/oem/usr/share/iqfiles","--silent",nullptr);
        perror("rkisp_demo");  // 找不到程序/动态库时，camera.log留下具体启动错误。
        _exit(127);
    }
    if (pid_<0) { error="无法启动MIPI采集"; stop(); return false; }
    return true;
}

bool Camera::read(cv::Mat& bgr, std::string& error)
{
    bgr.release();
    if (fd_<0 || pid_<=0) { error="摄像头尚未启动"; return false; }
    cv::Mat nv12(project::ImageHeight*3/2,project::ImageWidth,CV_8UC1);
    size_t received=0;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(15);
    while (received<nv12.total()) {
        if ((cancelled && cancelled()) || std::chrono::steady_clock::now()>=deadline) {
            error="采集取消或超时"; stop(); return false;
        }
        int status=0;
        const int child=waitpid(pid_,&status,WNOHANG);
        if (child==pid_ || (child<0 && errno==ECHILD)) {
            pid_=-1; error="MIPI采集进程退出，查看camera.log"; stop(); return false;
        }
        pollfd input{fd_,POLLIN,0};
        int ready=poll(&input,1,50);
        if (ready<0 && errno!=EINTR) { error="等待相机数据失败"; stop(); return false; }
        if (ready<=0) continue;
        if (input.revents & (POLLERR|POLLHUP|POLLNVAL)) { error="相机帧管道中断"; stop(); return false; }
        ssize_t count=::read(fd_,nv12.data+received,nv12.total()-received);
        if (count>0) received+=size_t(count);
        else if (count<0 && errno!=EINTR && errno!=EAGAIN) {
            error="读取相机数据失败"; stop(); return false;
        }
    }
    cv::cvtColor(nv12,bgr,cv::COLOR_YUV2BGR_NV12);
    return true;
}

bool Camera::stop()
{
    if (fd_>=0) { close(fd_); fd_=-1; }
    const bool captured=finish(pid_);
    const bool loaded=finish(loaderPid_);
    if (!captured || !loaded) return false;
    if (locked_) { unlink(fifo); rmdir(lock); locked_=false; }
    return true;
}

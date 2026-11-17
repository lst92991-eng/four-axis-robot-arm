//Candlelight 2.5 GS USB，沿用原实测适配器的配置和收发格式
#include "candlelight.hpp"
#include <libusb.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

namespace scara
{
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
static libusb_context *context;
static libusb_device_handle *usb;
static unsigned char epIn, epOut;
static int packetSize;
static uint32_t marker;
static bool claimed, elmue, paced;
static std::vector<uint8_t> received;
static Clock::time_point partialSince;

static void controlOut(uint8_t command, const std::vector<uint32_t> &words);
static void controlIn(uint8_t command, uint8_t *data, int length);
static void configureTiming(const uint8_t *caps, int offset, unsigned bitrate, uint8_t command);
static std::string usbString(libusb_device_handle *device, uint8_t index);

//---------初始化区---------
void Candle_open(const std::string &serial)
{
    if (libusb_init(&context) < 0) throw std::runtime_error("无法初始化libusb");
    try
    {
        libusb_device **devices;
        auto count = libusb_get_device_list(context, &devices);
        if (count < 0) throw std::runtime_error("无法枚举USB设备");
        auto freeDevices = [](libusb_device **list) { libusb_free_device_list(list, 1); };
        std::unique_ptr<libusb_device *, decltype(freeDevices)> deviceList(devices, freeDevices);
        unsigned matches = 0, detected = 0;
        int openError = 0;
        std::string product;
        for (decltype(count) i = 0; i < count; ++i)
        {
            libusb_device_descriptor descriptor{};
            libusb_get_device_descriptor(devices[i], &descriptor);
            if (!((descriptor.idVendor == 0x1d50 && descriptor.idProduct == 0x606f) ||
                  (descriptor.idVendor == 0x1209 && descriptor.idProduct == 0x2323))) continue;
            ++detected;
            libusb_device_handle *device;
            openError = libusb_open(devices[i], &device);
            if (openError < 0) continue;
            if ((serial.empty() || usbString(device, descriptor.iSerialNumber) == serial) && ++matches == 1)
            {
                usb = device;
                product = usbString(device, descriptor.iProduct);
            }
            else libusb_close(device);
        }
        if (serial.empty() && detected > 1)
            throw std::runtime_error("检测到多个USB-CAN，请用SCARA_CAN_SERIAL选择");
        if (matches != 1)
            throw std::runtime_error(openError == LIBUSB_ERROR_ACCESS ? "USB-CAN访问权限不足" :
                                     "请选择唯一的Candlelight USB-CAN，多个适配器时指定SCARA_CAN_SERIAL");
        //独占接口，避免旧面板和本程序同时发控制帧
        if (libusb_claim_interface(usb, 0) != 0)
            throw std::runtime_error("无法独占USB-CAN接口，请关闭旧面板或释放已占用的接口");
        claimed = true;

        libusb_config_descriptor *config;
        if (libusb_get_active_config_descriptor(libusb_get_device(usb), &config) != 0)
            throw std::runtime_error("无法读取USB-CAN端点");
        epIn = epOut = 0;
        if (config->bNumInterfaces && config->interface[0].num_altsetting)
        {
            const auto &alternate = config->interface[0].altsetting[0];
            for (int i = 0; i < alternate.bNumEndpoints; ++i)
            {
                const auto &ep = alternate.endpoint[i];
                if ((ep.bmAttributes & 3) != LIBUSB_TRANSFER_TYPE_BULK) continue;
                if (ep.bEndpointAddress & LIBUSB_ENDPOINT_IN) epIn = ep.bEndpointAddress;
                else { epOut = ep.bEndpointAddress; packetSize = ep.wMaxPacketSize; }
            }
        }
        libusb_free_config_descriptor(config);
        if (!epIn || !epOut || packetSize < 1) throw std::runtime_error("USB-CAN缺少数据端点");

        //先停CAN并清空旧会话残帧，再配置500k/2M和FD事务
        controlOut(2, {0, 0});
        auto deadline = Clock::now() + 1s;
        while (true)
        {
            uint8_t buffer[512];
            int length = 0;
            int rc = libusb_bulk_transfer(usb, epIn, buffer, sizeof(buffer), &length, 50);
            if (rc == LIBUSB_ERROR_TIMEOUT && length == 0) break;
            if ((rc < 0 && rc != LIBUSB_ERROR_TIMEOUT) || Clock::now() >= deadline)
                throw std::runtime_error("USB-CAN接收队列未清空");
        }
        controlOut(0, {0xbeef});
        uint8_t caps[72];
        controlIn(4, caps, 40);
        uint32_t features = readWord(caps);
        if ((features & 0x500) != 0x500) throw std::runtime_error("适配器不支持FD位时序");
        elmue = (features & 0x4000) != 0;
        paced = elmue && product.find("WeActStudioV1") != std::string::npos;
        configureTiming(caps, 8, 500000, 1);
        controlIn(11, caps, sizeof(caps));
        configureTiming(caps, 40, 2000000, 10);
        controlOut(2, {1, 0x100});
    }
    catch (...)
    {
        Candle_close();
        throw;
    }
}

//---------USB控制传输区---------
static std::string usbString(libusb_device_handle *device, uint8_t index)
{
    uint8_t buffer[128];
    int size = index ? libusb_get_string_descriptor_ascii(device, index, buffer, sizeof(buffer)) : 0;
    return size > 0 ? std::string(reinterpret_cast<char *>(buffer), size) : std::string{};
}

static void controlOut(uint8_t command, const std::vector<uint32_t> &words)
{
    std::vector<uint8_t> bytes(4 * words.size());
    for (size_t i = 0; i < words.size(); ++i) writeWord(bytes.data() + 4 * i, words[i]);
    int length = int(bytes.size());
    int rc = libusb_control_transfer(usb, 0x41, command, 0, 0, bytes.data(), length, 500);
    if (rc != length) throw std::runtime_error("USB-CAN配置失败，USB状态 " + std::to_string(rc));
    if (elmue)
    {
        uint8_t result = 0;
        controlIn(22, &result, 1);
        if (result != 2) throw std::runtime_error("USB-CAN拒绝配置，命令 " + std::to_string(command));
    }
}

static void controlIn(uint8_t command, uint8_t *data, int length)
{
    int rc = libusb_control_transfer(usb, 0xc1, command, 0, 0, data, length, 500);
    if (rc != length) throw std::runtime_error("USB-CAN能力读取失败，USB状态 " + std::to_string(rc));
}

static void configureTiming(const uint8_t *caps, int offset, unsigned bitrate, uint8_t command)
{
    //用设备返回的时钟、时间段和分频范围求精确位速率，采样点取最接近75%的组合
    unsigned clock = readWord(caps + 4), limits[8];
    for (int i = 0; i < 8; ++i) limits[i] = readWord(caps + offset + i * 4);
    unsigned count = 0;
    double error = 1;
    std::vector<uint32_t> best;
    for (unsigned brp = std::max(1u, limits[5]); brp <= std::min(limits[6], clock / bitrate);
         brp += std::max(1u, limits[7]))
    {
        if (clock % (brp * bitrate)) continue;
        unsigned total = clock / (brp * bitrate);
        if (total < 3) continue;
        for (unsigned s2 = limits[2]; s2 <= std::min(limits[3], total - 2); ++s2)
        {
            unsigned s1 = total - 1 - s2;
            if (s1 < limits[0] || s1 > limits[1]) continue;
            double diff = std::abs(double(1 + s1) / total - .75);
            if (diff > error || (diff == error && total <= count)) continue;
            error = diff;
            count = total;
            best = {0, s1, s2, std::min(s2, limits[4]), brp};
        }
    }
    if (best.empty()) throw std::runtime_error("USB-CAN无法生成所需位速率");
    controlOut(command, best);
}

//---------CAN帧收发区---------
void Candle_send(const CanFrame &frame)
{
    std::array<uint8_t, 76> bytes{};
    writeWord(bytes.data(), marker++ % 32);
    writeWord(bytes.data() + 4, frame.id);
    bytes[8] = 15;
    bytes[10] = 6; //FD和BRS
    std::copy(frame.data.begin(), frame.data.end(), bytes.begin() + 12);
    //原WeAct版本需要给64+12字节的第二个USB包留出端点重装时间
    int length = int(bytes.size());
    int chunk = paced ? packetSize : length;
    for (int pos = 0; pos < length; pos += chunk)
    {
        if (pos) std::this_thread::sleep_for(2ms);
        int size = std::min(chunk, length - pos), written = 0;
        int rc = libusb_bulk_transfer(usb, epOut, bytes.data() + pos, size, &written, 100);
        if (rc != 0 || written != size)
            throw std::runtime_error("USB-CAN发送未确认，USB状态 " + std::to_string(rc));
    }
}

bool Candle_receive(CanFrame &frame, unsigned timeoutMs)
{
    auto deadline = Clock::now() + std::chrono::milliseconds(timeoutMs);
    while (Clock::now() < deadline)
    {
        if (received.size() >= 12)
        {
            unsigned dlc = received[8];
            bool fd = (received[10] & 2) != 0;
            size_t size = fd ? 76 : 20;
            if (received[9] || dlc > (fd ? 15u : 8u)) throw std::runtime_error("USB-CAN帧头无效");
            if (received.size() >= size)
            {
                unsigned id = readWord(received.data() + 4);
                bool external = readWord(received.data()) == 0xffffffff;
                bool good = external && fd && dlc == 15 && id <= 0x7ff && (received[10] & 4);
                if (external && (id & 0x20000000))
                {
                    //过滤旧适配器无故障的错误计数/恢复通知
                    unsigned type = id & 0x1fffffff;
                    bool normal = !fd && dlc == 8 && (type == 0 || type == 4) && received[12] == 0 &&
                                  received[13] == (type == 4 ? 0x40 : 0) &&
                                  readWord(received.data() + 14) == 0 && std::max(received[18], received[19]) < 96;
                    if (!normal) throw std::runtime_error("CAN总线错误，类型 " + std::to_string(type));
                }
                if (good)
                {
                    frame.id = id;
                    std::copy_n(received.begin() + 12, 64, frame.data.begin());
                }
                received.erase(received.begin(), received.begin() + size);
                partialSince = Clock::now();
                if (good) return true;
                continue;
            }
        }
        //512字节读缓冲保留跨USB包的FD帧，直到凑齐一帧
        if (!received.empty() && Clock::now() - partialSince > 200ms)
            throw std::runtime_error("USB-CAN接收帧不完整");
        uint8_t buffer[512];
        int length = 0;
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
        int rc = libusb_bulk_transfer(usb, epIn, buffer, sizeof(buffer), &length, unsigned(std::max<int64_t>(1, remaining)));
        if (rc < 0 && rc != LIBUSB_ERROR_TIMEOUT)
            throw std::runtime_error("USB-CAN接收失败，USB状态 " + std::to_string(rc));
        if (received.empty()) partialSince = Clock::now();
        received.insert(received.end(), buffer, buffer + length);
    }
    return false;
}

//---------关闭区---------
void Candle_close() noexcept
{
    if (claimed)
    {
        uint8_t reset[8]{};
        libusb_control_transfer(usb, 0x41, 2, 0, 0, reset, sizeof(reset), 100);
        libusb_release_interface(usb, 0);
    }
    if (usb) libusb_close(usb);
    if (context) libusb_exit(context);
    usb = nullptr;
    context = nullptr;
    claimed = elmue = paced = false;
    received.clear();
    marker = 0;
}
}

#include "app/game_files.hpp"
#include "vision/io.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

volatile std::sig_atomic_t scara::gameRunning = 1;

static void check(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}

static void reply(const std::string &ticket, int settled, int success)
{
    cv::FileStorage file;
    scara::memoryStorage(true, file);
    file << "ticket" << ticket << "settled" << settled << "success" << success;
    auto path = scara::Runtime_path("motion-reply.json");
    scara::writeBytes(path + ".producer", file.releaseAndGetString());
    std::filesystem::rename(std::filesystem::u8path(path + ".producer"), std::filesystem::u8path(path));
}

int main()
{
    try
    {
        using namespace scara;
        Files_open("output/game-channel-test", true);
        std::string ticket;
        std::vector<PickStep> steps{{PickAction::MoveZ, {}, 123, 180}, {PickAction::ReleasePulse, {}, 0, 0, 20}};
        Motion_request("move", "a3a4", steps, ticket);
        reply("old-ticket", 1, 1);
        std::thread producer(
            [&]
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
                reply(ticket, 0, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                reply(ticket, 1, 1);
            });
        bool success = false;
        auto began = std::chrono::steady_clock::now();
        Motion_wait(ticket, success);
        producer.join();
        check(success && std::chrono::steady_clock::now() - began >= std::chrono::milliseconds(70),
              "Old or unsettled reply was accepted");
        cv::FileStorage request;
        readStorage(Runtime_path("motion-request.json"), request);
        check(int(request["simulated"]) == 1 && request["steps"].size() == 2, "Request contract incomplete");
        check((std::string)request["steps"][0]["action"] == "z" && double(request["steps"][0]["j1_deg"]) == 123,
              "J1 angle lost its unit or value");

        Motion_request("move", "a3a4", steps, ticket);
        reply(ticket, 1, 0);
        Motion_wait(ticket, success);
        check(!success, "Failure reply became success");
        Motion_request("move", "a3a4", steps, ticket);
        gameRunning = 0;
        bool stopped = false;
        try
        {
            Motion_wait(ticket, success);
        }
        catch (const std::exception &)
        {
            stopped = true;
        }
        cv::FileStorage cancel;
        readStorage(Runtime_path("motion-cancel.json"), cancel);
        check(stopped && (std::string)cancel["ticket"] == ticket && (std::string)cancel["action"] == "stop",
              "Cancellation did not identify the active transaction");
        gameRunning = 1;
        Motion_request("move", "a3a4", steps, ticket);
        cv::FileStorage malformed;
        memoryStorage(true, malformed);
        malformed << "ticket" << ticket << "settled" << 1;
        writeBytes(Runtime_path("motion-reply.json"), malformed.releaseAndGetString());
        bool rejected = false;
        try
        {
            Motion_wait(ticket, success);
        }
        catch (const std::exception &)
        {
            rejected = true;
        }
        check(rejected && Motion_cancel(), "Malformed completion was silently accepted");
        std::cout
            << "PASS unique request, stale/unsettled reply, explicit failure and cancellation; file-only simulation\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

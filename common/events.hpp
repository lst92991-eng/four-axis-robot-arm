//事件只向显示层交付原因及可选的棋盘差异，回调同步使用数据
#pragma once
#include "chess/board.hpp"
#include <array>

namespace scara
{
enum class EventType
{
    Vision, BoardChanged, IllegalMove, Motion, Placement, Engine, Stop, Restored, Application, Count
};
struct AppEvent
{
    EventType type;
    std::string message;
    const chess::Board *expected = nullptr;
    const chess::Board *observed = nullptr;
};
using EventCallback = void (*)(const AppEvent &);
inline std::array<EventCallback, size_t(EventType::Count)> eventCallbacks{};
inline void Event_on(EventType type, EventCallback callback)
{
    eventCallbacks[size_t(type)] = callback;
}
inline void Event_emit(EventType type, const std::string &message,
                       const chess::Board *expected = nullptr, const chess::Board *observed = nullptr)
{
    AppEvent event;
    event.type = type;
    event.message = message;
    event.expected = expected;
    event.observed = observed;
    EventCallback callback = eventCallbacks[size_t(type)];
    if (callback) callback(event);
}
} // namespace scara

// Release构建也执行输入、取消和恢复flags的验证。
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "app/commands.hpp"
#include <cassert>
#include <string>

void testCommands()
{
    int pipeEnds[2];
    assert(pipe(pipeEnds)==0);
    const int originalFlags=fcntl(pipeEnds[0],F_GETFL);
    {
        app::InputCommands commands(pipeEnds[0]);
        assert(fcntl(pipeEnds[0],F_GETFL)&O_NONBLOCK);
        commands.poll();
        assert(!commands.eof() && !commands.cancelled());
        const std::string first="move a0a1\npa";
        assert(write(pipeEnds[1],first.data(),first.size())==ssize_t(first.size()));
        commands.poll();
        std::string line;
        assert(commands.pop(line) && line=="move a0a1");
        assert(!commands.cancelled() && !commands.pop(line));
        assert(write(pipeEnds[1],"use\r\n",5)==5);
        commands.poll();
        assert(commands.cancelled());
        assert(commands.pop(line) && line=="pause" && !commands.cancelled());
        assert(write(pipeEnds[1],"new\nquit\n",9)==9);
        commands.poll();
        assert(commands.cancelled());
        assert(commands.pop(line) && line=="new" && commands.cancelled());
        assert(commands.pop(line) && line=="quit" && !commands.cancelled());

        const std::string pauseAfterMove="move a0a1\npause\n";
        assert(write(pipeEnds[1],pauseAfterMove.data(),pauseAfterMove.size())==ssize_t(pauseAfterMove.size()));
        commands.poll();
        assert(commands.pop(line) && line=="pause" && !commands.pop(line));
        const std::string newThenMove="move a0a1\nnew\nmove b0c2\n";
        assert(write(pipeEnds[1],newThenMove.data(),newThenMove.size())==ssize_t(newThenMove.size()));
        commands.poll();
        assert(commands.pop(line) && line=="new");
        assert(commands.pop(line) && line=="move b0c2" && !commands.pop(line));
        const std::string controls="scan\npause\nmove a0a1\nnew\nscan\nquit\n";
        assert(write(pipeEnds[1],controls.data(),controls.size())==ssize_t(controls.size()));
        commands.poll();
        assert(commands.pop(line) && line=="pause" && commands.cancelled());
        assert(commands.pop(line) && line=="new" && commands.cancelled());
        assert(commands.pop(line) && line=="quit" && !commands.pop(line));

        const std::string longest=std::string(80,'a')+'\n';
        assert(write(pipeEnds[1],longest.data(),longest.size())==ssize_t(longest.size()));
        commands.poll();
        assert(commands.pop(line) && line==std::string(80,'a') && !commands.eof());
        for (int i=0;i<8;++i) assert(write(pipeEnds[1],"scan\n",5)==5);
        commands.poll();
        for (int i=0;i<8;++i) assert(commands.pop(line) && line=="scan");
        assert(!commands.pop(line));
        close(pipeEnds[1]);
        commands.poll();
        assert(commands.eof() && commands.cancelled());
    }
    assert(fcntl(pipeEnds[0],F_GETFL)==originalFlags);
    close(pipeEnds[0]);

    // 超长行和队列超限都必须停止，不能静默截断成另一条命令。
    for (bool longLine : {true,false}) {
        assert(pipe(pipeEnds)==0);
        {
            app::InputCommands commands(pipeEnds[0]);
            std::string text=longLine ? std::string(81,'a') : std::string();
            if (!longLine) for (int i=0;i<9;++i) text+="scan\n";
            assert(write(pipeEnds[1],text.data(),text.size())==ssize_t(text.size()));
            commands.poll();
            std::string line;
            assert(commands.eof() && commands.cancelled() && !commands.pop(line));
        }
        close(pipeEnds[0]);
        close(pipeEnds[1]);
    }
    app::InputCommands invalid(-1);
    assert(invalid.eof() && invalid.cancelled());
}

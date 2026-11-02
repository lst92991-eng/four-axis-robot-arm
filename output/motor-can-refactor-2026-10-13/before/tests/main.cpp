#include <iostream>
void testAlgorithms();
void testChess();
void testSimulator();
void testCommands();
void testVision();
#ifdef SCARA_HAS_EMM_CAN
void testEmmCan();
#endif
int main()
{
    testAlgorithms();
    testChess();
    testSimulator();
    testCommands();
    testVision();
#ifdef SCARA_HAS_EMM_CAN
    testEmmCan();
#endif
    std::cout << "SCARA tests passed\n";
}

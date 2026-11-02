#include <iostream>
void testAlgorithms();
void testChess();
void testSimulator();
void testCommands();
void testVision();
#ifdef SCARA_HAS_MOTOR
void testMotor();
#endif
int main()
{
    testAlgorithms();
    testChess();
    testSimulator();
    testCommands();
    testVision();
#ifdef SCARA_HAS_MOTOR
    testMotor();
#endif
    std::cout << "SCARA tests passed\n";
}

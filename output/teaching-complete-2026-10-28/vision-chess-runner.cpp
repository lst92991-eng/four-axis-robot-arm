#include <exception>
#include <iostream>
void testVisionChess();
int main() {
    try { testVisionChess(); std::cout << "Vision/chess checks passed (OpenCV 5.0.0, real Pikafish).\n"; }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}

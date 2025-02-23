#include <iostream>
#include <fstream>
#include <string>
#include "A_star.h"
int main()
{
    std::shared_ptr<AStar> a_star = std::make_shared<AStar>();
    a_star->Excute();
    return 1;
}

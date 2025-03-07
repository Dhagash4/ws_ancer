#include <iostream>
#include <filesystem>
namespace fs = std::filesystem;

int main()
{
  fs::path p = "example.txt";
  std::cout << "Hello World!" << std::endl;
  return 0;
}
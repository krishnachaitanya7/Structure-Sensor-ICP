#include <iostream>
#include <thread>
#include <chrono>
using namespace std;
int main(){
    int count {0};
    while(1){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        cout << "The count is: " << count << flush;
        count++;

    }

}
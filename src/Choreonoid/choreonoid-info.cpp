#include <cnoid/ExecutablePath>
#include <iostream>

using namespace std;
using namespace cnoid;

int main(int argc, char *argv[])
{
    if(argc == 2 && string(argv[1]) == "--share-directory"){
        cout << shareDir() << endl;
    }
    return 0;
}

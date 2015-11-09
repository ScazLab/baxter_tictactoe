#include "src/utils/T_ThreadSafe.h"
#include <iostream>

/*! \file test_thread_safe_variable.cpp
    \brief Test for ThreadSafeVariable class
    \author Álvaro Castro González, acgonzal@ing.uc3m.es
    \date created long long time ago...

    This is a simpe testing program for the ThreadSafeVariable class.
    An thread safe integer is created, a new value is set, and this value is printed out on the console.
 */


int main(){
    ThreadSafeVariable<int> safe_i;
    safe_i.set(12);
    std::cout << "New value set to " << safe_i.get() << std::endl;
    return 0;
}

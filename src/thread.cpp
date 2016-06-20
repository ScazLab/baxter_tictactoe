#include <pthread.h>
#include <unistd.h>
#include <cstdio>
#include <iostream>
#include <cstdlib>

using namespace std;

/*STACK OVERFLOW CODE: ONLY USE FOR TESTING PURPOSES*/
class MyThreadClass
{
public:
   MyThreadClass() {/* empty */}
   virtual ~MyThreadClass() {/* empty */}

   /** Returns true if the thread was successfully started, false if there was an error starting the thread */
   bool StartInternalThread()
   {
      return (pthread_create(&_thread, NULL, InternalThreadEntryFunc, this) == 0);
   }

   /** Will not return until the internal thread has exited. */
   void WaitForInternalThreadToExit()
   {
      (void) pthread_join(_thread, NULL);
   }

protected:
   /** Implement this method in your subclass with the code you want your thread to run. */
   virtual void InternalThreadEntry() = 0;

private:
   static void * InternalThreadEntryFunc(void * This) {((MyThreadClass *)This)->InternalThreadEntry(); return NULL;}

   pthread_t _thread;
};

class Fibonacci : public MyThreadClass
{

protected:
    void InternalThreadEntry()
    {
        int fib_a = 0;
        int fib_b = 1;
        for(int i = 0; i < 40; i++)
        {
            int fib_c = fib_a + fib_b;
            fib_a = fib_b;
            fib_b = fib_c;
            cout << "[thread] " << fib_c << endl;
        }
    }  
};

int main(int argc, char const *argv[])
{
    Fibonacci * fib = new Fibonacci();
    fib->StartInternalThread();
    for(int i = 0; i > -100; i--)
    {
        if(i == -50)
        {
            fib->WaitForInternalThreadToExit();
        }
        cout << "[main] " << i << endl;
    }

    return 0;
}
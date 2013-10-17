#ifndef T_THREADSAFE_H
#define T_THREADSAFE_H

/*! \file T_ThreadSafe.h
    \brief Creation of thread save variables
    \author Álvaro Castro González, acgonzal@ing.uc3m.es
    \date created long long time ago...
    \class ThreadSafeVariable

    This is a template class for creating thread safe variables.
    This means that you can create a variable which guarantees safe access from several threads.
    The type of the variable can be any which can be accessed (set and get) through the operator =
 */

#include <pthread.h>

template<typename T>
class ThreadSafeVariable{
private:

    //! The mutex.
    /*!
        Mutex for controlling the access to the value of the thread safe variable.
    */
    pthread_mutex_t mutex;

    //! The variable itself.
    /*!
        The thread safe variable itself.
    */
    T value;
public:

    //! Constructor.
    /*!
        Constructor of the class. The variable is not initialized.
    */
    ThreadSafeVariable(){
        pthread_mutex_init(&this->mutex, NULL);
    }

    //! Get function.
    /*!
        This function returns the value of the variable.
        It is thread safe accessed.
        \return A copy of the current value of the variable
    */
    T get(){
        T aux;
        pthread_mutex_lock(&this->mutex);
        aux=value;
        pthread_mutex_unlock(&this->mutex);
        return aux;
    }

    //! Set function.
    /*!
        This function sets a new value to the variable.
        It is thread safe accessed.
        \param newValue The new value that the variable will be set to
    */
    void set(T newValue){
        pthread_mutex_lock(&this->mutex);
        this->value = newValue;
        pthread_mutex_unlock(&this->mutex);
    }

};

#endif // T_THREADSAFE_H

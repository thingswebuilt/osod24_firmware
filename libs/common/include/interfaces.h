//
// Created by robbe on 06/01/2024.
//

#ifndef COMMAND_H
#define COMMAND_H
#include "types.h"

namespace COMMON {

    // Observer interface
    class Observer {
    protected:
        ~Observer() = default;

    public:
        virtual void update(DriveTrainState newState) = 0;
    };


    // Subject interface
    class Subject {
    protected:
        ~Subject() = default;

    public:
        virtual void addObserver(Observer* observer) = 0;
        virtual void notifyObservers(DriveTrainState newState) = 0;
    };

} // COMMON

#endif //COMMAND_H

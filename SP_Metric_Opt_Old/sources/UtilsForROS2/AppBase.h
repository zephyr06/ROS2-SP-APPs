#pragma once
#include "string"

class AppBase {
   public:
    AppBase(std::string app_name) : app_name_(app_name) {}
    // function arguments not used for now
    virtual void run(int) { ; }

    // data member
    std::string app_name_;
};
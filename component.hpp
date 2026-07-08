#pragma once

class Component {
public:
    virtual bool setup() = 0;
    virtual void update() = 0;
    virtual void calib() = 0;
};
#pragma once

class button_t
{
private:
    bool oldVal = false;
public:
    bool update(bool newVal)
    {
        bool retVal = false;
        if (!oldVal && newVal)
            retVal = true;
        oldVal = newVal;
        return retVal;
    }
};

class toggle_t
{
private:
    button_t button;
    bool toggleVal;
public:
    toggle_t(bool initialVal = false) : toggleVal(initialVal) {};
    bool update(bool newVal)
    {
        bool retVal = button.update(newVal);
        if (retVal)
            toggleVal = !toggleVal;
        return retVal;
    }
    bool getToggleVal()
    {
        return toggleVal;
    }
};
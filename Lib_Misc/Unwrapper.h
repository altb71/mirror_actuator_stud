/*  
*/

using namespace std;

class Unwrapper
{
public:

    Unwrapper(double);
    
    double operator()(short inc) {
        return doStep(inc);
    }
    
    virtual     ~Unwrapper();
    
    void        reset(void);
    double       doStep(short inc);

private:

    long last_value;
    double inc2rad;

};

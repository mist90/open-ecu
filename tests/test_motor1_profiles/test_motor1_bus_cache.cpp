#include "profile_math.hpp"
#include <cassert>
using namespace libecu;
int main(){
    using M=FixedMath;using V=FixedProfile::Voltage;
    auto value=[](int n){return number<V>(41*n,1770);};
    for(int n=1536;n<1792;++n)M::warmBus(value(n));
    auto equal=[](V v){assert(M::preparedInverse(v)==M::inverseBusVoltage(v));};
    for(int n=0;n<4096;++n) {
        V v=value(n);
        for(int delta:{-3,-1,0,1,3})equal(V::fromRaw(v.raw()+delta));
    }
    // Force a collision: only the exact key is allowed to use the new reciprocal.
    for(int k=0;k<512;k++){
        auto a=V::fromRaw((5<<21)+k*4096+2000),b=V::fromRaw(a.raw()+(1<<21));
        M::warmBus(a);equal(a);equal(b);
        M::warmBus(b);equal(a);equal(b);
    }
    for(int32_t v:{INT32_MIN,-1,0,262143,262144,262145,INT32_MAX})equal(V::fromRaw(v));
}

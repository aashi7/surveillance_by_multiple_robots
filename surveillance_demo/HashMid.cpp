#include <vector>
#include <boost/functional/hash.hpp>

using namespace std;

/* Hash function for vector of bools */
class HashMid
{
public:
    size_t operator()(vector<bool> const& vec) const {
        size_t num = 0;
        for(int i = 0; i < vec.size(); i++)
            num += vec[i]*pow(2,i);
        return num;
        //return boost::hash_range(vec.begin(), vec.end());
    }
};
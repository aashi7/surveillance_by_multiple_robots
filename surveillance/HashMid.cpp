#include <vector>
#include <boost/functional/hash.hpp>

using namespace std;

/* Hash function for vector of bools */
class HashMid
{
public:
    size_t operator()(vector<bool> const& vec) const {
        return boost::hash_range(vec.begin(), vec.end());
    }
};
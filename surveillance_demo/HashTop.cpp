#include <vector>
#include <boost/functional/hash.hpp>

using namespace std;

/* Hash function for vector of integers */
class HashTop
{
public:
    size_t operator()(vector<int> const& vec) const {
        return boost::hash_range(vec.begin(), vec.end());
    }
};
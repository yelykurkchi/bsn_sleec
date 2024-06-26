#include "libbsn/generator/MarkovString.hpp"

using namespace std;
using namespace bsn::range;

namespace bsn {
    namespace generator {
        MarkovString::MarkovString() : transitions(), currentState(), states() {}

        // Construtor
        MarkovString::MarkovString(array<float,25> t, array<Range, 5> r, int32_t initialState) :
            transitions(t),
            currentState(initialState),
            states(r) {}

        MarkovString::MarkovString(const MarkovString &obj) :
            transitions(obj.transitions),
            currentState(obj.currentState),
            states(obj.states) {}
        
        MarkovString& MarkovString::operator=(const MarkovString &obj) {
            transitions = obj.transitions;
            currentState = obj.currentState;
            states = obj.states;
            return (*this);
        }

        const string MarkovString::toString() const {
            stringstream sstr;

            sstr << "MarkovString " << "" << endl;

            return sstr.str();
        }

    }
}

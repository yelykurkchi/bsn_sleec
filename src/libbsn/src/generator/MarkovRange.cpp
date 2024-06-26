#include "libbsn/generator/MarkovRange.hpp"

using namespace std;
using namespace bsn::range;

namespace bsn {
    namespace generator {
        MarkovRange::MarkovRange() : transitions(), currentState(), states() {}

        // Construtor
        MarkovRange::MarkovRange(array<float,25> t, array<Range, 5> r, int32_t initialState) :
            transitions(t),
            currentState(initialState),
            states(r) {}

        MarkovRange::MarkovRange(const MarkovRange &obj) :
            transitions(obj.transitions),
            currentState(obj.currentState),
            states(obj.states) {}
        
        MarkovRange& MarkovRange::operator=(const MarkovRange &obj) {
            transitions = obj.transitions;
            currentState = obj.currentState;
            states = obj.states;
            return (*this);
        }

        const string MarkovRange::toString() const {
            stringstream sstr;

            sstr << "MarkovRange " << "" << endl;

            return sstr.str();
        }

    }
}

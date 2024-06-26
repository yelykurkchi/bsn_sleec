#ifndef MARKOV_HPP
#define MARKOV_HPP

#include <iostream>
#include <array> 
#include <random>
#include <vector>
#include <stdint.h>
#include <sstream>

#include "libbsn/range/Range.hpp"

namespace bsn {
    namespace generator {

        class MarkovString {
            public:
                MarkovString();
                
                MarkovString(std::array<float, 25> transitions, std::array<std:string, 5> states, int32_t initialState);

                MarkovString(const MarkovString & /*obj*/);
                MarkovString& operator=(const MarkovString & /*obj*/);

                // Contém a probabilidade de todas as transições
                std::array<float,25> transitions;
                // 0 para low 1 para medium e 2 para high
                int32_t currentState;
                // Contém os intervalos de cada estado
                std::array<std:string, 5> states;

                // Calcula o próximo estado da cadeia de markov
                void next_state();

                const std::string toString() const;
        };

    }

}

#endif
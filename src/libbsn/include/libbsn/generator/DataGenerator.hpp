#ifndef DATAGENERATOR_HPP
#define DATAGENERATOR_HPP

#include "libbsn/generator/MarkovRange.hpp"

#include <random>
#include <stdint.h>

namespace bsn {
    namespace generator {
        class DataGenerator {
            public:
                DataGenerator();
                DataGenerator(const MarkovRange& markov);
                DataGenerator(const DataGenerator& obj);
                ~DataGenerator();

                DataGenerator& operator=(const DataGenerator& obj);
                
                double getValue();
                void setSeed();
                void nextState();

            private:
                double calculateValue();

                MarkovRange markovChain;
                double value;
                std::mt19937 seed;
        };
    }
}

#endif
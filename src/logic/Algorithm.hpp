#ifndef ALGORITHM_H
#define ALGORITHM_H

namespace logic
{
class Algorithm
{
public:
    virtual ~Algorithm() {};

    template<typename input, typename output>
    output process(input in);
};
}

#endif // ALGORITHM_H

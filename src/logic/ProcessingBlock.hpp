#ifndef PROCESSINGBLOCK_H
#define PROCESSINGBLOCK_H

#include "Algorithm.hpp"
#include "LogicCommons.hpp"

namespace logic
{
template<typename input, typename output>
class ProcessingBlock : public Algorithm
{
public:
	virtual ~ProcessingBlock()
	{
		if (processing_block != nullptr)
		{
			delete processing_block;
		}
	}

	template<typename internal_input, typename internal_output>
	void set_processing_block(ProcessingBlock<internal_input, internal_output> &pb)
	{
		processing_block = &pb;
	}

	virtual output process(input in) = 0;
	ProcessingBlock* get_processing_block()
	{
		return processing_block;
	}
protected:
	Algorithm *processing_block = nullptr;
};
}

#endif // PROCESSINGBLOCK_H

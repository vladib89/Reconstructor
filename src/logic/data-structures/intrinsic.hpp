#ifndef INTRINSIC
#define INTRINSIC

namespace logic
{
	namespace data_structures
	{
		enum model
		{
			DISTORTION_MODIFIED_BROWN_CONRADY,
			DISTORTION_INVERSE_BROWN_CONRADY,
			DISTORTION_BROWN_CONRADY,
			DISTORTION_FTHETA,
			DISTORTION_KANNALA_BRANDT4
		};

		struct intrinsic
		{
			int width;
			int height;
			bool distorted = false;
			double fx;
			double fy;
			double cx;
			double cy;
			model model;
			float* distortion = nullptr;
		};
	}
}

#endif // !INTRINSIC

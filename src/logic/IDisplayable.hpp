#ifndef IDISPLAYABLE_HPP
#define IDISPLAYABLE_HPP

namespace logic
{
	class IDisplayable
	{
	public:
		virtual bool set_image(void** img) = 0;
	};
} // namespace logic

#endif // !IDISPLAYABLE_HPP

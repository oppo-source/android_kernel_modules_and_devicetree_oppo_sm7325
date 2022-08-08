vendor := $(srctree)/$(src)

#ifndef VENDOR_EDIT
#ifneq "$(wildcard $(vendor)/qcom)" ""
#	subdir-y += qcom
#endif
#else
	subdir-y += qcom
	subdir-y += oplus/peacock
	subdir-y += oplus/libai
	subdir-y += oplus/fluorite
	subdir-y += oplus/martini
#endif

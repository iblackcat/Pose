#ifndef DATASET_HELPER_H
#define DATASET_HELPER_H

#include "utils.h"

namespace mf {

extern const GlobalCoeff G640;
extern const GlobalCoeff G320;

class DatasetHelper {
public:
	DatasetHelper() {}

	bool writeImage(const void const *i, const char * file_name, int channels = 4);
	bool writeModel(u32 *C, u32 *SW, int index);
	//Frame readFrame(int index, int channels = 4);
	void *readImage(const char *filepath, int channels = 4);

private:
	char path[256];
};



} //namespace mf

#endif //DATASET_HELPER_H

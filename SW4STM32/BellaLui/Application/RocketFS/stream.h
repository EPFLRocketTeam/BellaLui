   /*
 * stream.h
 *
 *  Created on: 17 Oct 2019
 *      Author: Arion
 */

#ifndef INC_STREAM_H_
#define INC_STREAM_H_

#include <stdint.h>
#include <stdbool.h>

#include "filesystem.h"

bool init_stream(Stream* stream, FileSystem* filesystem, uint32_t base_address, FileType type);

#endif /* INC_STREAM_H_ */

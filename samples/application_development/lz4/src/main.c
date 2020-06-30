/*
 * Copyright (c) 2020 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <lz4.h>

void main(void)
{
	const char *src = "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Quisque "
	"sodales lorem lorem, sed congue enim vehicula a. Sed finibus diam sed "
	"odio ultrices pharetra. Nullam dictum arcu ultricies turpis congue, "
	"vel venenatis turpis venenatis. Nam tempus arcu eros, ac congue libero "
	"tristique congue. Proin velit lectus, euismod sit amet quam in, "
	"maximus condimentum urna. Cras vel erat luctus, mattis orci ut, varius "
	"urna. Nam eu lobortis velit."
	"\n"
	"Nullam sit amet diam vel odio sodales cursus vehicula eu arcu. Proin "
	"fringilla, enim nec consectetur mollis, lorem orci interdum nisi, "
	"vitae suscipit nisi mauris eu mi. Proin diam enim, mollis ac rhoncus "
	"vitae, placerat et eros. Suspendisse convallis, ipsum nec rhoncus "
	"aliquam, ex augue ultrices nisl, id aliquet mi diam quis ante. "
	"Pellentesque venenatis ornare ultrices. Quisque et porttitor lectus. "
	"Ut venenatis nunc et urna imperdiet porttitor non laoreet massa. Donec "
	"eleifend eros in mi sagittis egestas. Sed et mi nunc. Nunc vulputate, "
	"mauris non ullamcorper viverra, lorem nulla vulputate diam, et congue "
	"dui velit non erat. Duis interdum leo et ipsum tempor consequat. In "
	"faucibus enim quis purus vulputate nullam."
	"\n";

	const int src_size = (int)(strlen(src) + 1);
	const int max_dst_size = LZ4_compressBound(src_size);

	printk("max_dst_size: %d \n", max_dst_size);

	char *compressed_data = malloc((size_t)max_dst_size);

	if (compressed_data == NULL) {
		printk("Failed to allocate memory for compressed data \n");
		return;
	}

	printk("Memory allocated for compressed data \n");

	const int compressed_data_size = LZ4_compress_default(src, compressed_data,
					 src_size, max_dst_size);

	printk("compressed_data_size: %d \n", compressed_data_size);

	if (compressed_data_size <= 0) {
		printk("Failed to compress the data \n");
		return;
	}

	if (compressed_data_size > 0) {
		printk("successfully compressed data! Ratio: %.2f\n",
		       (float) compressed_data_size/src_size);
	}

	compressed_data = (char *)realloc(compressed_data, (size_t)compressed_data_size);
	if (compressed_data == NULL) {
		printk("Failed to re-alloc memory for compressed data \n");
		return;
	}

	char* const decompressed_data = malloc(src_size);

	if (decompressed_data == NULL) {
		printk("Failed to allocate memory to decompress data \n");
		return;
	}

	const int decompressed_size = LZ4_decompress_safe(compressed_data,
				      decompressed_data, compressed_data_size,
				      src_size);
	free(compressed_data);

	if (decompressed_size < 0) {
		printk("Failed to decompress the data \n");
		return;
	}

	if (decompressed_size >= 0) {
		printk("Successfully decompressed some data \n");
	}

	if (decompressed_size != src_size) {
		printk("Decompressed data is different from original \n");
	}

	if (memcmp(src, decompressed_data, src_size) != 0) {
		printk("Validation failed. *src and *new_src are not identical \n");
		return;
	}

	printk("Validation done. The string we ended up with is: \n%s\n",
			decompressed_data);
}

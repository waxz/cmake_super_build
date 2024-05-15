//
// Created by waxz on 4/10/24.
//

#include "absl_hash_seed_test.h"
#include "absl/types/variant.h"
#include "absl/container/flat_hash_map.h"

#include <iostream>

void absl_hash_seed_test::hash(int v) {

    // Hash the integer using absl::Hash
    size_t hashValue = absl::Hash<int>{}(v);

    // Print the hash value
    std::cout << "absl_hash_seed_test Hash value of " << v << ": " << hashValue << std::endl;

}
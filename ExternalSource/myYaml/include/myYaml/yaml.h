#ifndef YAML_H_62B23520_7C8E_11DE_8A39_0800200C9A66
#define YAML_H_62B23520_7C8E_11DE_8A39_0800200C9A66

#if defined(_MSC_VER) ||                                            \
    (defined(__GNUC__) && (__GNUC__ == 3 && __GNUC_MINOR__ >= 4) || \
     (__GNUC__ >= 4))  // GCC supports "pragma once" correctly since 3.4
#pragma once
#endif

#include "ExternalSource/myYaml/include/myYaml/parser.h"
#include "ExternalSource/myYaml/include/myYaml/emitter.h"
#include "ExternalSource/myYaml/include/myYaml/emitterstyle.h"
#include "ExternalSource/myYaml/include/myYaml/stlemitter.h"
#include "ExternalSource/myYaml/include/myYaml/exceptions.h"

#include "ExternalSource/myYaml/include/myYaml/node/node.h"
#include "ExternalSource/myYaml/include/myYaml/node/impl.h"
#include "ExternalSource/myYaml/include/myYaml/node/convert.h"
#include "ExternalSource/myYaml/include/myYaml/node/iterator.h"
#include "ExternalSource/myYaml/include/myYaml/node/detail/impl.h"
#include "ExternalSource/myYaml/include/myYaml/node/parse.h"
#include "ExternalSource/myYaml/include/myYaml/node/emit.h"
#include "ExternalSource/myYaml/include/myYaml/yaml_eigen.h"

#endif  // YAML_H_62B23520_7C8E_11DE_8A39_0800200C9A66

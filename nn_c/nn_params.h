#ifndef __NEURAL_NET_PARAMS_H__
#define __NEURAL_NET_PARAMS_H__

#ifdef __cpluscplus
extern "C" {
#endif


const float W_0[2][2] = {{0.05182042, 0.12931477},  {-0.05775117, -0.14651702}};

const float b_0[2] = {-0.025308503, -0.17890204};

const float W_1[2][6] = {{ 0.54260856, -0.02306426,  0.19087929, -0.74251944, -0.63831645, -0.68327546},
                        {-0.04570496, -0.7770299 , -0.10920978,  0.5312753 , -0.45460272, -0.6480367 }};

const float b_1[6] = {-0.16165425, 0.0, 0.0, -0.1908861, 0.0, 0.0};

const float W_2[6][12] = {{ 0.5345626 ,  0.27464488,  0.53279865, -0.44043735, -0.23395373,
                       -0.38969666,  0.2896213 ,  0.32518882, -0.14963828, -0.37089354,
                        0.5856703 ,  0.3403039 },  {-0.5476972 , -0.32107654,  0.26390123,  0.45594382,  0.42738783,
                       -0.20959365,  0.32754445,  0.2189166 ,  0.44781065,  0.4667325 ,
                        0.39535534,  0.49131083},  { 0.37024385,  0.39850444, -0.15958229, -0.33447796, -0.3154433 ,
                       -0.24923179,  0.20335793, -0.51533765,  0.36057782, -0.4796366 ,
                       -0.0408175 , -0.48231378},  {-0.5271103 ,  0.07002232,  0.3826115 ,  0.41982663,  0.46532595,
                       -0.06497443, -0.56828296, -0.12047482,  0.11633596, -0.0730108 ,
                       -0.35926244, -0.39117432},  { 0.5593059 ,  0.51852536, -0.24737018,  0.5093626 , -0.02761543,
                       -0.17861846, -0.33134776,  0.35856414,  0.13757038,  0.3354649 ,
                       -0.05058843,  0.40767473},  {-0.0514186 ,  0.00354618,  0.10182732, -0.16634479, -0.13842738,
                        0.35791028, -0.01864898,  0.55499697, -0.35078174,  0.13070762,
                       -0.488994  , -0.30015123}};

const float b_2[12] = {0.07999225, -0.18308885, 0.072582416, -0.021493308, 0.09034264, 0.0, -0.031766012, -0.09135227, 0.09885199, 0.0, -0.0678337, 0.06452919};;

const float W_3[12][4] = {{ 0.01211957, -0.39770052, -0.1866196 , -0.47824368},
                            {0.29413274, 0.18162274, 0.22416595, 0.47284794},
                            { 0.19953065, -0.30839902,  0.07550647,  0.0540893 },
                            {-0.49706087,  0.19446188, -0.00699842,  0.34173045},
                            {-0.10066439, -0.4475149 , -0.46172065,  0.23057945},
                            { 0.24928832, -0.45265847,  0.53113765,  0.24870843},
                            { 0.44186038,  0.09189403, -0.13527969,  0.48199195},
                            { 0.07205318, -0.5879065 ,  0.6361132 , -0.10520257},
                            {-0.16457032, -0.17022297, -0.49656427,  0.42314428},
                            {-0.4605224 , -0.17327249, -0.12314001, -0.156079  },
                            {-0.29106447,  0.0764299 ,  0.0652269 , -0.1218145 },
                            { 0.2794138 ,  0.3066044 ,  0.4288032 , -0.18838462}};

const float b_3[4] = {0.08243948, 0.0, -0.08753328, -0.09023327};

const float W_4[4][1] = {{0.20900065},  {-0.30000144},  {-0.11763141},  {-0.09080353}};

const float b_4[1] = {0.08473853};


#ifdef __cplusplus
}

#endif
#endif
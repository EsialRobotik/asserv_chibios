#ifndef SAMPLE_STREAM_SRC_DATASTREAMTYPE_MACROS_H_
#define SAMPLE_STREAM_SRC_DATASTREAMTYPE_MACROS_H_

#include <vector>
#include <string>

/* Utils to generate call to function FOR_EACH_TUPLE_DESCRIPTION_xx */ 
#define CONCATENATE(arg1, arg2)   CONCATENATE1(arg1, arg2)
#define CONCATENATE1(arg1, arg2)  CONCATENATE2(arg1, arg2)
#define CONCATENATE2(arg1, arg2)  arg1##arg2

/* NUM_TUPLE count the number of variable argument passed */
#define NUM_TUPLE_HELPER(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, _48, _49, _50, _51, _52, _53, _54, _55, _56, _57, _58, _59, _60, _61, _62, _63, _64, _65, _66, _67, _68, _69, _70, _71, _72, _73, _74, _75, _76, _77, _78, _79, _80, _81, _82, _83, _84, _85, _86, _87, _88, _89, _90, _91, _92, _93, _94, _95, _96, _97, _98, _99, _100, N, ...)    N
#define NUM_TUPLE(...)      NUM_TUPLE_HELPER(__VA_ARGS__, 50, 0, 49, 0, 48, 0, 47, 0, 46, 0, 45, 0, 44, 0, 43, 0, 42, 0, 41, 0, 40, 0, 39, 0, 38, 0, 37, 0, 36, 0, 35, 0, 34, 0, 33, 0, 32, 0, 31, 0, 30, 0, 29, 0, 28, 0, 27, 0, 26, 0, 25, 0, 24, 0, 23, 0, 22, 0, 21, 0, 20, 0, 19, 0, 18, 0, 17, 0, 16, 0, 15, 0, 14, 0, 13, 0, 12, 0, 11, 0, 10, 0, 9, 0, 8, 0, 7, 0, 6, 0, 5, 0, 4, 0, 3, 0, 2, 0, 1, 0, 0)


/* 
 *  First part :
 *    generate the structure that will contains each sample of a loop.
 *    The struct will look like this :
 * 
    typedef struct
    {
        uint32_t synchro;
        uint32_t sample_size_without_synchro;
        float array[xx]
    }__attribute__((packed)) UsbStreamSample;
 */ 
#define CREATE_STRUCT( ... ) \
uint32_t const array_nb_element =  NUM_TUPLE(__VA_ARGS__);  \
                                          \
typedef struct{                           \
    uint32_t synchro;                     \
    uint32_t sample_size_without_synchro_nor_this; \
    float array[NUM_TUPLE(__VA_ARGS__)];  \
} __attribute__((packed)) UsbStreamSample;



/* 
 *  Second part :
 *    Generate the functions to fill the structure created in the first part.
 *    Fill function will look like this :
         inline void SpeedGoalRight (float val) { m_currentStruct.array[2] = val;} ;
 */ 

#define FOR_EACH_TUPLE_ACCESS_FUNC_1(X, desc)         \
    inline void set## X (float val) { setValue( &(m_currentStruct.array[0]), val);} ;
    
#define FOR_EACH_TUPLE_ACCESS_FUNC_2( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_1(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_3( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_2(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_4( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_3(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_5( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_4(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_6( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_5(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_7( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_6(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_8( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_7(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_9( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_8(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_10( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_9(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_11( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_10(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_12( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_11(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_13( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_12(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_14( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_13(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_15( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_14(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_16( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_15(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_17( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_16(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_18( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_17(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_19( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_18(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_20( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_19(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_21( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_20(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_22( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_21(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_23( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_22(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_24( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_23(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_25( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_24(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_26( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_25(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_27( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_26(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_28( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_27(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_29( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_28(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_30( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_29(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_31( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_30(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_32( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_31(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_33( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_32(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_34( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_33(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_35( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_34(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_36( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_35(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_37( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_36(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_38( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_37(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_39( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_38(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_40( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_39(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_41( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_40(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_42( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_41(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_43( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_42(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_44( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_43(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_45( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_44(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_46( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_45(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_47( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_46(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_48( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_47(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_49( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_48(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_50( X, desc, ...)    \
      FOR_EACH_TUPLE_ACCESS_FUNC_49(__VA_ARGS__)      \
      inline void set## X (float val) { setValue( &(m_currentStruct.array[NUM_TUPLE(__VA_ARGS__)]), val);} ;

#define FOR_EACH_TUPLE_ACCESS_FUNC_(N, ...) CONCATENATE(FOR_EACH_TUPLE_ACCESS_FUNC_, N)( __VA_ARGS__)
#define FOR_EACH_TUPLE_ACCESS_FUNC( ...) FOR_EACH_TUPLE_ACCESS_FUNC_(NUM_TUPLE(__VA_ARGS__),  __VA_ARGS__)


#define CREATE_ACCESS_FUNC( ... )   \
     FOR_EACH_TUPLE_ACCESS_FUNC( __VA_ARGS__ )     

     



/* 
 *  Last part :
 *    Generate a string that aggregate all the description of the sampled informations.
 *    Theses descriptions will be transmitted to plotjuggled
 */ 

#define STRING_COMMA(s)  #s","
#define FOR_EACH_TUPLE_DESCRIPTION_1(X, desc)         \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_2( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_1(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_3( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_2(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_4( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_3(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_5( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_4(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_6( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_5(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_7( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_6(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_8( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_7(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_9( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_8(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_10( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_9(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_11( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_10(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_12( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_11(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_13( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_12(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_14( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_13(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_15( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_14(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_16( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_15(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_17( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_16(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_18( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_17(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_19( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_18(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_20( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_19(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_21( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_20(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_22( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_21(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_23( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_22(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_24( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_23(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_25( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_24(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_26( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_25(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_27( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_26(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_28( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_27(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_29( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_28(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_30( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_29(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_31( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_30(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_32( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_31(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_33( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_32(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_34( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_33(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_35( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_34(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_36( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_35(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_37( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_36(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_38( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_37(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_39( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_38(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_40( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_39(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_41( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_40(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_42( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_41(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_43( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_42(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_44( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_43(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_45( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_44(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_46( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_45(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_47( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_46(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_48( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_47(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_49( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_48(__VA_ARGS__)    \
    STRING_COMMA(desc)

#define FOR_EACH_TUPLE_DESCRIPTION_50( X,  desc, ...)    \
    FOR_EACH_TUPLE_DESCRIPTION_49(__VA_ARGS__)    \
    STRING_COMMA(desc)
                              
    


#define FOR_EACH_TUPLE_DESC_(N, ...) CONCATENATE(FOR_EACH_TUPLE_DESCRIPTION_, N)( __VA_ARGS__)
#define FOR_EACH_TUPLE_DESC( ...) FOR_EACH_TUPLE_DESC_(NUM_TUPLE(__VA_ARGS__),  __VA_ARGS__)

#define CREATE_DESCRIPTION( ... ) \
                                    \
    char const * description = { \
    FOR_EACH_TUPLE_DESC(__VA_ARGS__) \
    };


/* 
 *  Entry point function :
 *     call this function like this :

     GENERATE_USB_STREAM_HEADER(
        SpeedGoalRight,     "speed/right/goal",
        SpeedEstimatedRight,"speed/right/current",
        OdoX,  "odometry/X",
        OdoY,  "odometry/Y");
 */ 

#define GENERATE_USB_STREAM_HEADER( ... ) \
    CREATE_STRUCT(__VA_ARGS__)            \
    CREATE_ACCESS_FUNC(__VA_ARGS__)       \
    CREATE_DESCRIPTION(__VA_ARGS__)


#endif /* USBSTREAM_SRC_DATASTREAMTYPE_MACROS_H_ */

#ifndef _TYPEDEFS_H
#define _TYPEDEFS_H

#ifndef BOOL
typedef unsigned char   BOOL;
#endif

typedef unsigned char   UINT8;
typedef unsigned int    UINT32;
typedef signed int      INT32;

#if 0
typedef unsigned short  UINT16;
typedef unsigned short  USHORT;
typedef signed char     INT8;
typedef signed short    INT16;
typedef unsigned int    DWORD;
typedef void            VOID;
typedef unsigned char   BYTE;
typedef float           FLOAT;
#endif

typedef unsigned char MUINT8;
typedef unsigned short MUINT16;
typedef unsigned int MUINT32;
typedef unsigned long long MUINT64;

typedef signed char MINT8;
typedef signed short MINT16;
typedef signed int MINT32;
typedef signed long long MINT64;

typedef float MFLOAT;

typedef void MVOID;
typedef bool MBOOL;

#ifndef MTRUE
#define MTRUE               1
#endif

#ifndef MFALSE
#define MFALSE              0
#endif

#ifndef TRUE
#define TRUE                1
#endif

#ifndef FALSE
#define FALSE               0
#endif

#endif

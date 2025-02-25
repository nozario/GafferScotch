#ifndef GAFFERSCOTCH_EXPORT_H
#define GAFFERSCOTCH_EXPORT_H

#ifdef _MSC_VER
#ifdef GafferScotch_EXPORTS
#define GAFFERSCOTCH_API __declspec(dllexport)
#else
#define GAFFERSCOTCH_API __declspec(dllimport)
#endif
#else
#define GAFFERSCOTCH_API
#endif

#endif // GAFFERSCOTCH_EXPORT_H
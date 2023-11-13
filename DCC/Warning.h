// Warning.h: Implementation of warning message

#ifndef WARNING_H
#define WARNING_H

#pragma once

#define DBG_WARN(message) (AfxMessageBox(_T(message)))
#define DBG_WARN_AND_RETURN_VOID_UNLESS(condition, message) if (!(condition)) { \
	AfxMessageBox(_T(message)); \
	return; }
#define DBG_WARN_AND_RETURN_UNLESS(condition, retVal, message) if (!(condition)) { \
	AfxMessageBox(_T(message)); \
	return (retVal); }

#endif
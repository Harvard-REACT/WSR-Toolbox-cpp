//
// Created by Weiying Wang on 12/21/20.
//

#ifndef CSITOOLBOX_ERROR_H
#define CSITOOLBOX_ERROR_H

namespace ct{
#define THROW_CSI_INVALID_ARGUMENT_ERROR(msg) ct::Error::throwError<std::invalid_argument>(__FILE__, __func__, __LINE__, msg)
#define THROW_CSI_RUNTIME_ERROR(msg) ct::Error::throwError<std::runtime_error>(__FILE__, __func__, __LINE__, msg)
    class Error {
    public:
        template<typename eType>
        static void throwError(const std::string& file,
                        const std::string& function,
                        std::uint32_t line,
                        const std::string& msg = "")
        {
            std::string errMsg = "File: " + file + "\n\tFunction: " + function + "\n\tLine: " + std::to_string(line) + "\n\tError: " + msg;
            std::cerr << errMsg;
            throw eType(errMsg);
        }
    };
}


#endif //CSITOOLBOX_ERROR_H

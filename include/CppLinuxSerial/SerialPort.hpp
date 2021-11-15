///
/// \file 			SerialPort.hpp
/// \author 		Geoffrey Hunter <gbmhunter@gmail.com> (www.mbedded.ninja)
/// \created		2014-01-07
/// \last-modified 	2019-05-30
/// \brief			The main serial port class.
/// \details
///					See README.rst in repo root dir for more info.

// Header guard
#ifndef SERIAL_PORT_SERIAL_PORT_H
#define SERIAL_PORT_SERIAL_PORT_H

// System headers
#include <string>
#include <fstream> // For file I/O (reading/writing to COM port)
#include <sstream>
// #include <termios.h> // POSIX terminal control definitions (struct termios)
// #include <asm/termios.h> // Terminal control definitions (struct termios)
#include <vector>
#include <asm/ioctls.h>
#include <asm/termbits.h>

// User headers
#include "Exception.hpp"

namespace mn {
    namespace CppLinuxSerial {

        /// \brief      Represents the baud rate "types" that can be used with the serial port. STANDARD represents all
        ///             the standard baud rates as provided by UNIX, CUSTOM represents a baud rate defined by an arbitray integer.
        enum class BaudRateType {
            STANDARD,
            CUSTOM,
        };

        /// \brief		Strongly-typed enumeration of baud rates for use with the SerialPort class
        /// \details    Specifies all the same baud rates as UNIX, as well as B_CUSTOM to specify your
        ///             own. See https://linux.die.net/man/3/cfsetispeed for list of supported UNIX baud rates.
        enum class BaudRate {
            B_0,
            B_50,
            B_75,
            B_110,
            B_134,
            B_150,
            B_200,
            B_300,
            B_600,
            B_1200,
            B_1800,
            B_2400,
            B_4800,
            B_9600,
            B_19200,
            B_38400,
            B_57600,
            B_115200,
            B_230400,
            B_460800,
            B_CUSTOM, // Placeholder 
        };

        /// \brief      Represents the state of the serial port.
        enum class State {
            CLOSED,
            OPEN,
        };

/// \brief		SerialPort object is used to perform rx/tx serial communication.
        class SerialPort {

        public:
            /// \brief		Default constructor. You must specify at least the device before calling Open().
            SerialPort();

            /// \brief		Constructor that sets up serial port with the basic (required) parameters.
            SerialPort(const std::string &device, BaudRate baudRate);

            /// \brief		Constructor that sets up serial port with the basic (required) parameters.
            SerialPort(const std::string &device, speed_t baudRate);

            /// \brief		Destructor. Closes serial port if still open.
            virtual ~SerialPort();

            /// \brief		Sets the device to use for serial port communications.
            /// \details    Method can be called when serial port is in any state.
            void SetDevice(const std::string &device);

            /// \brief      Allows the user to set a standard baud rate.
            void SetBaudRate(BaudRate baudRate);

            /// \brief      Allows the user to set a custom baud rate.
            void SetBaudRate(speed_t baudRate);

            /// \brief      Sets the read timeout (in milliseconds)/blocking mode.
            /// \details    Only call when state != OPEN. This method manupulates VMIN and VTIME.
            /// \param      timeout_ms  Set to -1 to infinite timeout, 0 to return immediately with any data (non
            ///             blocking, or >0 to wait for data for a specified number of milliseconds). Timeout will
            ///             be rounded to the nearest 100ms (a Linux API restriction). Maximum value limited to
            ///             25500ms (another Linux API restriction).
            void SetTimeout(int32_t timeout_ms);

            /// \brief		Enables/disables echo.
            /// \param		value		Pass in true to enable echo, false to disable echo.
            void SetEcho(bool value);

            /// \brief		Opens the COM port for use.
            /// \throws		CppLinuxSerial::Exception if device cannot be opened.
            /// \note		Must call this before you can configure the COM port.
            void Open();

            /// \brief		Closes the COM port.
            void Close();

            /// \brief		Sends a message over the com port.
            /// \param		data		The data that will be written to the COM port.
            /// \throws		CppLinuxSerial::Exception if state != OPEN.
            void Write(const std::string& data);

            /// \brief		Use to read from the COM port.
            /// \param		data		The object the read characters from the COM port will be saved to.
            /// \param      wait_ms     The amount of time to wait for data. Set to 0 for non-blocking mode. Set to -1
            ///                 to wait indefinitely for new data.
            /// \throws		CppLinuxSerial::Exception if state != OPEN.
            void Read(std::string& data);

            // unused
            // ssize_t readch(int fd, char *ptr);
            // ssize_t readline(int fd, void *vptr, size_t maxlen);
            // void ReadLine(std::string& data);

            ssize_t ReadByte(unsigned char& charBuffer, const size_t msTimeout = 0 );
            void ReadLine(std::string& dataString, 
                            char lineTerminator = '\n',
                            size_t msTimeout = 0);

        private:

            /// \brief		Returns a populated termios structure for the passed in file descriptor.
            // termios GetTermios();

            /// \brief		Configures the tty device as a serial port.
            /// \warning    Device must be open (valid file descriptor) when this is called.
            void ConfigureTermios();

            // void SetTermios(termios myTermios);

            /// \brief		Returns a populated termios2 structure for the serial port pointed to by the file descriptor.
            termios2 GetTermios2();

            /// \brief      Assigns the provided tty settings to the serial port pointed to by the file descriptor.
            void SetTermios2(termios2 tty);

            /// \brief      Keeps track of the serial port's state.
            State state_;

            /// \brief      The file path to the serial port device (e.g. "/dev/ttyUSB0").
            std::string device_;

            /// \brief      The type of baud rate that the user has specified.
            BaudRateType baudRateType_;

            /// \brief      The current baud rate if baudRateType_ == STANDARD.
            BaudRate baudRateStandard_;

            /// \brief      The current baud rate if baudRateType_ == CUSTOM.
            speed_t baudRateCustom_;

            /// \brief		The file descriptor for the open file. This gets written to when Open() is called.
            int fileDesc_;

            bool echo_;

            int32_t timeout_ms_;

            std::vector<char> readBuffer_;
            unsigned int readBufferSize_B_;

            static constexpr BaudRate defaultBaudRate_ = BaudRate::B_57600;
            static constexpr int32_t defaultTimeout_ms_ = -1;
            static constexpr unsigned int defaultReadBufferSize_B_ = 255;


        };

        /**
        * Type-safe and portable equivalent of TEMP_FAILURE_RETRY macro that is
        * provided gcc. See
        * https://www.gnu.org/software/libc/manual/html_node/Interrupted-Primitives.html
        * and signal(7) man-page for details. The purpose of this function is to
        * repeat system calls that are interrupted and set errno to EINTR.
        * Typically, POSIX applications that use signal handlers must check for
        * EINTR after each library function that can return it in order to try the
        * call again. Often programmers forget to check, which is a common source
        * of error. This may also happen in multi-threaded applications. For
        * example, see
        * https://www.linuxquestions.org/questions/programming-9/problem-in-creating-serial-port-application-using-threads-869149/#post4299011
        *
        * As an example of usage of this function, consider the following use of
        * open() system call:
        *
        * @code{.cpp}
        * const auto fd = open("foo.txt", O_WRONLY | O_APPEND);
        * @endcode
        *
        * If this system call is interrupted by a signal, it will return -1 and
        * errno will be set to EINTR. In order to retry this system call on
        * such interruptions, replace the above call with the following:
        *
        * @code{.cpp}
        * const auto fd = call_with_retry(open, "foo.txt", O_WRONLY | O_APPEND);
        * @endcode
        *
        * @param func
        *     The function to be called and retried on EINTR. This function is
        *     expected to return -1 and set errno to EINTR in case of a system
        *     call interruption.
        *
        * @param args
        *     The arguments to be passed to the function.
        *
        * @return
        *     The value returned by the function after it completes without
        *     interruption that sets errno to EINTR.
        */
        template<typename Fn, typename... Args>
        typename std::result_of<Fn(Args...)>::type
        call_with_retry(Fn func, Args... args)
        {
            using result_type = typename std::result_of<Fn(Args...)>::type ;
            result_type result ;
            do {
                result = func(std::forward<Args>(args)...);
            } while((result == -1) and (errno == EINTR)) ;
            return result ;
        }

        /**
        * @brief Exception error thrown when data could not be read from the
        *        serial port before the timeout had been exceeded.
        */
        class ReadTimeout : public std::runtime_error
        {
        public:
            /**
            * @brief Exception error thrown when data could not be read from the
            *        serial port before the timeout had been exceeded.
            */
            explicit ReadTimeout(const std::string& whatArg)
                : runtime_error(whatArg)
            {
            }
        } ;

    } // namespace CppLinuxSerial
} // namespace mn

#endif // #ifndef SERIAL_PORT_SERIAL_PORT_H

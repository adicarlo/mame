// license:BSD-3-Clause
// copyright-holders:Olivier Galibert, R. Belmont, Vas Crabb
//============================================================
//
//  sdlptty_unix.c - SDL pseudo tty access functions
//
//  SDLMAME by Olivier Galibert and R. Belmont
//
//============================================================

#include "posixfile.h"

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <climits>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdlib>

#if defined(__FreeBSD__) || defined(__DragonFly__)
#include <termios.h>
#include <libutil.h>
#elif defined(__NetBSD__) || defined(__OpenBSD__) || defined(__APPLE__)
#include <termios.h>
#include <util.h>
#elif defined(__linux__) || defined(__EMSCRIPTEN__)
#include <pty.h>
#elif defined(__HAIKU__)
#include <bsd/pty.h>
#endif


namespace {

#if defined(__APPLE__)
char const *const posix_ptty_identifier  = "/dev/pty";
#else
char const *const posix_ptty_identifier  = "/dev/pts";
#endif


class posix_osd_ptty : public osd_file
{
public:
	posix_osd_ptty(posix_osd_ptty const &) = delete;
	posix_osd_ptty(posix_osd_ptty &&) = delete;
	posix_osd_ptty& operator=(posix_osd_ptty const &) = delete;
	posix_osd_ptty& operator=(posix_osd_ptty &&) = delete;

	posix_osd_ptty(int fd, bool packet) noexcept : m_fd(fd), m_packet(packet), m_stopped(false)
	{
		assert(m_fd >= 0);
	}

	virtual ~posix_osd_ptty()
	{
		::close(m_fd);
	}

	virtual std::error_condition read(void *buffer, std::uint64_t offset, std::uint32_t count, std::uint32_t &actual) noexcept override
	{
		if (!m_packet)
		{
			ssize_t const result = ::read(m_fd, buffer, count);
			if (result < 0)
				return std::error_condition(errno, std::generic_category());

			actual = std::uint32_t(size_t(result));
			return std::error_condition();
		}

#if defined(TIOCPKT)
		// In packet mode a read returns a status byte on its own, or a zero byte followed by
		// data.  While the slave's own flow control (IXON) has its output stopped, hold back
		// what it wrote before stopping, as a serial port stops sending: a one-byte read
		// takes a pending status, or only the zero byte, leaving the data queued.
		std::uint8_t packet[1 + 256];
		std::size_t const length(m_stopped ? 1 : (1 + std::min<std::size_t>(count, sizeof(packet) - 1)));
		ssize_t const result = ::read(m_fd, packet, length);
		if (result < 0)
			return std::error_condition(errno, std::generic_category());

		actual = 0;
		if (result > 0)
		{
			if (packet[0] == TIOCPKT_DATA)
			{
				actual = std::uint32_t(size_t(result) - 1);
				std::memcpy(buffer, &packet[1], actual);
			}
			else if (packet[0] & TIOCPKT_STOP)
			{
				m_stopped = true;
			}
			else if (packet[0] & TIOCPKT_START)
			{
				m_stopped = false;
			}
		}
#endif
		return std::error_condition();
	}

	virtual std::error_condition write(void const *buffer, std::uint64_t offset, std::uint32_t count, std::uint32_t &actual) noexcept override
	{
		ssize_t const result = ::write(m_fd, buffer, count);
		if (result < 0)
			return std::error_condition(errno, std::generic_category());

		actual = std::uint32_t(size_t(result));
		return std::error_condition();
	}

	virtual std::error_condition truncate(std::uint64_t offset) noexcept override
	{
		// doesn't make sense on ptty
		return std::errc::bad_file_descriptor;
	}

	virtual std::error_condition flush() noexcept override
	{
		// no userspace buffers on read/write
		return std::error_condition();
	}

private:
	int m_fd;
	bool m_packet;      // master is in packet mode
	bool m_stopped;     // slave output stopped by its flow control
};

} // anonymous namespace


bool posix_check_ptty_path(std::string const &path) noexcept
{
	return strncmp(path.c_str(), posix_ptty_identifier, strlen(posix_ptty_identifier)) == 0;
}


std::error_condition posix_open_ptty(std::uint32_t openflags, osd_file::ptr &file, std::uint64_t &filesize, std::string &name) noexcept
{
#if defined(__ANDROID__)
	return std::errc::not_supported; // TODO: revisit this error code
#else // defined(__ANDROID__)
	struct termios tios;
	std::memset(&tios, 0, sizeof(tios));
	tios.c_iflag = 0;
	tios.c_oflag = 0;
	tios.c_cflag = CS8;
	tios.c_lflag = 0;

	int masterfd = -1, slavefd = -1;
#if defined(TTY_NAME_MAX)
	// TTY_NAME_MAX and ptsname_r were added to Open Group Base Specifications Issue 8
	if (::openpty(&masterfd, &slavefd, nullptr, &tios, nullptr) < 0)
		return std::error_condition(errno, std::generic_category());

	::close(slavefd);

	char slavepath[TTY_NAME_MAX + 1];
	auto const result = ::ptsname_r(masterfd, slavepath, std::size(slavepath));
	if (result == -1)
	{
		// pre-standard implementations of ptsname_r (e.g. FreeBSD, Tru64, HP-UX) return -1 and set errno
		std::error_condition err(errno, std::generic_category());
		::close(masterfd);
		return err;
	}
	else if (result != 0)
	{
		::close(masterfd);
		return std::error_condition(result, std::generic_category());
	}
#elif defined(__linux__) || defined(__FreeBSD__)
	// ptsname_r is present but there's no maximum length defined
	if (::openpty(&masterfd, &slavefd, nullptr, &tios, nullptr) < 0)
		return std::error_condition(errno, std::generic_category());

	::close(slavefd);

	std::vector<char> slavepath_storage;
	try
	{
		int result;
		do
		{
			if (slavepath_storage.empty())
				slavepath_storage.resize(PATH_MAX);
			else
				slavepath_storage.resize(slavepath_storage.size() * 2);
			result = ::ptsname_r(masterfd, slavepath_storage.data(), slavepath_storage.size());
			if (result == -1)
				result = errno; // pre-standard ptsname_r returns -1 and sets errno
		}
		while (result == ERANGE);
		if (result != 0)
		{
			::close(masterfd);
			return std::error_condition(result, std::generic_category());
		}
	}
	catch (...)
	{
		::close(masterfd);
		return std::errc::not_enough_memory;
	}
	char const *const slavepath = slavepath_storage.data();
#else
	// using openpty with a non-null slave path is considered unsafe
#if defined(PATH_MAX)
	char slavepath[PATH_MAX];
#else // defined PATH_MAX
	char slavepath[8192];
#endif // defined PATH_MAX
	if (::openpty(&masterfd, &slavefd, slavepath, &tios, nullptr) < 0)
		return std::error_condition(errno, std::generic_category());

	::close(slavefd);
#endif

	int const oldflags = ::fcntl(masterfd, F_GETFL, 0);
	if (oldflags < 0)
	{
		std::error_condition err(errno, std::generic_category());
		::close(masterfd);
		return err;
	}

	if (::fcntl(masterfd, F_SETFL, oldflags | O_NONBLOCK) < 0)
	{
		std::error_condition err(errno, std::generic_category());
		::close(masterfd);
		return err;
	}

	// packet mode reports the slave's flow control stopping and starting its output; carry
	// on without it where it isn't supported
	bool packet = false;
#if defined(TIOCPKT)
	int const on = 1;
	packet = ::ioctl(masterfd, TIOCPKT, &on) == 0;
#endif

	try
	{
		name = slavepath;
		file = std::make_unique<posix_osd_ptty>(masterfd, packet);
		filesize = 0;
		return std::error_condition();
	}
	catch (...)
	{
		::close(masterfd);
		return std::errc::not_enough_memory;
	}
#endif // defined(__ANDROID__)
}

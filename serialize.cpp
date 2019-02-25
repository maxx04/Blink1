#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

typedef unsigned char byte_t;
typedef std::vector<byte_t> buffer;

std::size_t object_size(const char* s) {
	return std::strlen(s);
};

template<typename T>
std::size_t object_size(T const& obj) {
	return sizeof(obj);
};

template<typename T>
buffer serialize(const T& obj) {
	std::size_t size = object_size(obj);
	buffer buf(size);

	byte_t const* obj_begin = reinterpret_cast<byte_t const*>(&obj);
	std::copy(obj_begin, obj_begin + size, buf.begin());

	return buf;
};

template<>
buffer serialize<std::string>(std::string const& str) {
	return serialize(str.c_str());
};

template<typename T>
T deserialize(buffer const& buf) {
	return *reinterpret_cast<const T*>(&buf[0]);
}

template<>
std::string deserialize<std::string>(buffer const& buf) {
	return deserialize<char*>(buf);
}

int main() {
	using std::cout;

	int x = 97;
	buffer c = serialize(x);
	cout << deserialize<int>(c) << "\n";

	char g = 'g';
	buffer c2 = serialize(g);
	cout << deserialize<char>(c2) << "\n";

	std::string k = "blabla";
	buffer c3 = serialize(k.c_str());
	cout << deserialize<char*>(c3) << "\n";

	std::string ka = "string";
	buffer c4 = serialize(ka);
	cout << deserialize<std::string>(c4) << "\n";
}
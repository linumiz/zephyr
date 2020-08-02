/* Taken from https://gist.github.com/alan-mushi/19546a0e2c6bd4e059fd */

#include <json.h>
#include <zephyr.h>
#include <sys/printk.h>

int main()
{
	struct json_object *jobj;
	char *str = "{ \"msg-type\": [ \"0xdeadbeef\", \"irc log\" ], \
		     \"msg-from\": { \"class\": \"soldier\", \"name\": \"Wixilav\" }, \
		     \"msg-to\": { \"class\": \"supreme-commander\", \"name\": \"[Redacted]\" }, \
		     \"msg-log\": [ \
		     \"soldier: Boss there is a slight problem with the piece offering to humans\", \
		     \"supreme-commander: Explain yourself soldier!\", \
		     \"soldier: Well they don't seem to move anymore...\", \
		     \"supreme-commander: Oh snap, I came here to see them twerk!\" \
		     ] \
		}";

	printk("str:\n---\n%s\n---\n\n", str);

	jobj = json_tokener_parse(str);
	printk("jobj from str:\n---\n%s\n---\n", json_object_to_json_string_ext(jobj, JSON_C_TO_STRING_SPACED | JSON_C_TO_STRING_PRETTY));

	return 0;
}

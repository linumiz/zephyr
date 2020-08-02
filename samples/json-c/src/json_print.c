/* Taken from https://gist.github.com/alan-mushi/19546a0e2c6bd4e059fd */

#include <json.h>
#include <zephyr.h>
#include <sys/printk.h>
 
int main(void)
{
	struct json_object *jobj;
	const char *question = "Mum, clouds hide alien spaceships don't they ?";
	const char *answer = "Of course not! (\"sigh\")";
	int i;
	struct {
		int flag;
		const char *flag_str;
	} json_flags[] = {
		{ JSON_C_TO_STRING_PLAIN, "JSON_C_TO_STRING_PLAIN" },
		{ JSON_C_TO_STRING_SPACED, "JSON_C_TO_STRING_SPACED" },
		{ JSON_C_TO_STRING_PRETTY, "JSON_C_TO_STRING_PRETTY" },
		{ JSON_C_TO_STRING_NOZERO, "JSON_C_TO_STRING_NOZERO" },
		{ JSON_C_TO_STRING_SPACED | JSON_C_TO_STRING_PRETTY, "JSON_C_TO_STRING_SPACED | JSON_C_TO_STRING_PRETTY" },
		{ -1, NULL }
	}; // Create an anonymous struct, instanciate an array and fill it

	printk("Using printk(): \"%s\", \"%s\"\n\n", question, answer);
	printk("Using json_object_to_json_string_ext():\n");

	/*
	 * The following create an object and add the question and answer to it.
	 */
	jobj = json_object_new_object();
	json_object_object_add(jobj, "question", json_object_new_string(question));
	json_object_object_add(jobj, "answer", json_object_new_string(answer));

	for (i = 0; json_flags[i].flag_str != NULL; i++) {
		printk("\nFlag %s:\n---\n", json_flags[i].flag_str);
		printk("%s\n---\n", json_object_to_json_string_ext(jobj, json_flags[i].flag));
	}

	json_object_put(jobj); // Delete the json object

	return 0;
}

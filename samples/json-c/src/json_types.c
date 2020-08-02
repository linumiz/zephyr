/* Taken from https://gist.github.com/alan-mushi/19546a0e2c6bd4e059fd */

#include <json.h>
#include <zephyr.h>
#include <sys/printk.h>

int main(void)
{
	struct json_object *array, *object, *tmp, *secu_code;
	char *val_type_str, *str;
	int val_type, i;

	array = json_object_new_array();
	object = json_object_new_object();

	// Fill the array
	tmp = json_object_new_int(12345);
	json_object_array_add(array, tmp);
	tmp = json_object_new_boolean(true);
	json_object_array_put_idx(array, 1, tmp);

	// Fill the object
	tmp = json_object_new_string("We have been made!");
	json_object_object_add(object, "message", tmp);
	json_object_object_add(object, "security-code", array);

	array = tmp = NULL; // "Nothing in the sleeves"
	str = NULL;

	// key and val don't exist outside of this bloc
	json_object_object_foreach(object, key, val) {
		printk("key: \"%s\", type of val: ", key);
		val_type = json_object_get_type(val);

		switch (val_type) {
			case json_type_null:
				val_type_str = "val is NULL";
				break;

			case json_type_boolean:
				val_type_str = "val is a boolean";
				break;

			case json_type_double:
				val_type_str = "val is a double";
				break;

			case json_type_int:
				val_type_str = "val is an integer";
				break;

			case json_type_string:
				val_type_str = "val is a string";
				str = (char *) json_object_get_string(val);
				break;

			case json_type_object:
				val_type_str = "val is an object";
				break;

			case json_type_array:
				val_type_str = "val is an array";
				break;
		}

		printk("%s", val_type_str);

		if (str)
			printk("\t->\t\"%s\"", str);

		printk("\n");
		str = NULL;
	}

	printk("\nDetails of the security code:\n");
	// Get the json_object associated to the "security-code" key in secu_code
	json_object_object_get_ex(object, "security-code", &secu_code);

	// For each case of the secu_code array
	for (i = 0; i < json_object_array_length(secu_code); i++) {
		// Set in tmp the json_object of the secu_code array at index i
		tmp = json_object_array_get_idx(secu_code, i);
		printk("security-code[%d] = %s\n", i, json_object_to_json_string(tmp));
	}

	printk("\nJson in plain text: \n");
	printk("---\n%s\n---\n", json_object_to_json_string(object));
	json_object_put(object);

	return 0;
}

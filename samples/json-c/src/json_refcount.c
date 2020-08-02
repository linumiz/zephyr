/* Taken from https://gist.github.com/alan-mushi/19546a0e2c6bd4e059fd */

#include <json.h>
#include <zephyr.h>
#include <sys/printk.h>

#define KEY1	"peace-code"
#define KEY2	"death-ray-code"
/*
 * Just a utility function.
 */
void print_json_object(struct json_object *jobj, const char *msg)
{
	printk("\n%s: \n", msg);
	printk("---\n%s\n---\n", json_object_to_json_string(jobj));
}

/*
 * In real life the operations done here are way more complicated, but it's
 * only an example.
 */
struct json_object * find_something(struct json_object *jobj, const char *key)
{
	struct json_object *tmp;

	json_object_object_get_ex(jobj, key, &tmp);

	return tmp;
}

/*
 * Swap the first and second elements of the array received.
 * Again, more complicated in real life...
 */
void glitch_in_the_matrix(struct json_object *jobj)
{
	struct json_object *pos1, *pos2;

	print_json_object(jobj, "Before the glitch in the matrix");

	pos1 = json_object_array_get_idx(jobj, 0);
	pos2 = json_object_array_get_idx(jobj, 1);

	// json_object_array_put_idx decrement the refcount if you replace an
	// element, but we don't whant that.
	json_object_array_put_idx(jobj, 0, json_object_get(pos2));
	json_object_array_put_idx(jobj, 1, json_object_get(pos1));

	print_json_object(jobj, "After the glitch in the matrix");
}

int main(void)
{
	struct json_object *obj1, *obj2, *res, *sub_obj1, *sub_obj2, *tmp;

	// Creation and print of obj1 and obj2
	obj1 = json_object_new_object();
	obj2 = json_object_new_object();
	json_object_object_add(obj1, KEY1, json_object_new_int(1234));
	json_object_object_add(obj2, KEY2, json_object_new_int(4321));
	print_json_object(obj1, "obj1 in plaintext");
	print_json_object(obj2, "obj2 in plaintext");

	// Extraction of the interesting parts of obj1 and obj2
	sub_obj1 = find_something(obj1, "peace-code");
	sub_obj2 = find_something(obj2, "death-ray-code");

	res = json_object_new_array();

	// We are going to use those in res so we need to increment the
	// refcount
	json_object_array_add(res, json_object_get(sub_obj1));
	json_object_array_add(res, json_object_get(sub_obj2));

	glitch_in_the_matrix(res);

	tmp = json_object_array_get_idx(res, 0);
	printk("\n[>] Unlocking peace with code at index 0 of res: %d\n", json_object_get_int(tmp));

	// We can safely put all of these objects
	json_object_put(obj1);
	json_object_put(obj2);
	json_object_put(res);

	return 0;
}

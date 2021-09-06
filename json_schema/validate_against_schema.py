import jsonref
import os
from jsonschema import validate
import sys


def main():
    if len(sys.argv) < 2:
        print("Please provide the filepath to be validated as an arg.")
        return

    current_directory = os.path.abspath(os.path.dirname(__file__))
    schema_file_path = os.path.join(current_directory, "schema.json")

    with open(schema_file_path) as schema_file:
        my_schema = jsonref.load(schema_file, jsonschema=True)

        file_path = sys.argv[1]
        with open(file_path) as json_file:
            validate(instance=jsonref.load(json_file), schema=my_schema)


if __name__ == "__main__":
    main()

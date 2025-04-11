# Adapted from https://github.com/autowarefoundation/autoware.universe/blob/b415e2e/mkdocs_macros.py
# Licensed under Apache 2.0


# This file is for defining macros for mkdocs-macros plugin
# Check https://mkdocs-macros-plugin.readthedocs.io/en/latest/macros/ for the details

import json
import os

from tabulate import tabulate


def format_param_type(param_type):
    if param_type == "number":
        return "float"
    else:
        return param_type


def ensure_word_breaks(text):
    """For URLs/paths, insert HTML word break marks. Does not affect other or non-string inputs."""
    if not isinstance(text, str):
        return text

    return text.replace("/", "/<wbr>")


def format_param_range(param):
    list_of_range = []
    if "enum" in param.keys():
        list_of_range.append(", ".join(map(str, param["enum"])))
    if "minimum" in param.keys():
        list_of_range.append("≥ " + str(param["minimum"]))
    if "exclusiveMinimum" in param.keys():
        list_of_range.append("> " + str(param["exclusiveMinimum"]))
    if "maximum" in param.keys():
        list_of_range.append("≤ " + str(param["maximum"]))
    if "exclusiveMaximum" in param.keys():
        list_of_range.append("< " + str(param["exclusiveMaximum"]))
    if "exclusive" in param.keys():
        list_of_range.append("≠ " + str(param["exclusive"]))

    if len(list_of_range) == 0:
        return "N/A"
    else:
        range_in_text = ""
        for item in list_of_range:
            if range_in_text != "":
                range_in_text += "<br/>"
            range_in_text += str(item)
        return range_in_text


def extract_parameter_info(
    parameters: dict,
    namespace="",
    file_directory: str = "",
    include_refs=False,
):
    params = []
    for k, v in parameters.items():
        v: dict

        # If the parameter definition references another place, include the referenced information
        if "$ref" in v.keys():
            if not include_refs:
                continue

            # Example: path/to/file.json#/json/path/to/parameter
            ref: str = v["$ref"]
            ref_path, ref_json_path = ref.split("#")
            ref_path = os.path.join(file_directory, ref_path)
            ref_json_path = ref_json_path.split("/")
            if ref_json_path[0] == "":
                ref_json_path = ref_json_path[1:]

            # Get param from referenced file
            with open(ref_path) as f:
                data = json.load(f)
            param = get_json_path(data, ref_json_path)

            # Then override param fields with possible extra information provided in `v`
            param.update({k: v for k, v in v.items() if k != "$ref"})

            # Wrap the parameter in a dictionary that is accepted by `extract_parameter_info`
            param_dict = {k: param}

            # Extract the doctored parameter
            extracted_from_ref = extract_parameter_info(
                param_dict, namespace, os.path.split(ref_path)[0], include_refs
            )

            params.extend(extracted_from_ref)

        elif v["type"] != "object":
            param = {}
            param["Name"] = namespace + k
            param["Type"] = format_param_type(v["type"])
            param["Description"] = v.get("description", "")
            param["Default"] = ensure_word_breaks(v.get("default", ""))
            param["Range"] = format_param_range(v)
            params.append(param)
        else:  # if the object is namespace, then dive deeper in to json value
            params.extend(
                extract_parameter_info(v["properties"], k + ".", file_directory, include_refs)
            )
    return params


def get_json_path(json_data: dict, json_path: list):
    for elem in json_path:
        if isinstance(elem, int):
            json_data = list(json_data.values())[elem]
        else:
            json_data = json_data[elem]
    return json_data


def format_json(json_data):
    # cspell: ignore tablefmt
    markdown_table = tabulate(json_data, headers="keys", tablefmt="github")
    return markdown_table


def define_env(env):
    @env.macro
    def json_to_markdown(
        json_schema_file_path,
        json_path=["definitions", 0, "properties"],
        include_refs=True,
    ):
        with open(json_schema_file_path) as f:
            data = json.load(f)

        params = get_json_path(data, json_path)
        param_info = extract_parameter_info(
            params,
            file_directory=os.path.split(json_schema_file_path)[0],
            include_refs=include_refs,
        )
        return format_json(param_info)

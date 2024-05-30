import os
import subprocess
import glob
import colorama

colorama.init(autoreset=True, strip=False)

def main():
    validation_failed = False

    for schema_path in glob.glob('./**/schema/*.schema.json', recursive=True):
        schema_file = os.path.relpath(schema_path, './')
        base_name = os.path.basename(schema_file).replace('.schema.json', '')
        config_dir = os.path.dirname(schema_file).replace('schema', 'config')

        str_indentation = ' ' * 4
        config_files = glob.glob(f'{config_dir}/**/{base_name}*.param.yaml', recursive=True)
        if not config_files:
            print(colorama.Fore.YELLOW + f'{str_indentation}No configuration files found for schema {schema_file}.')
            continue

        for config_file in config_files:
            print(
                colorama.Style.BRIGHT + '🔍 Validating: ' +
                colorama.Style.RESET_ALL +
                colorama.Fore.CYAN + f'{config_file} ' +
                colorama.Style.RESET_ALL + '🆚 ' +
                colorama.Fore.BLUE + f'{schema_file}' +
                colorama.Style.RESET_ALL,
                end=' '
            )
            result = subprocess.run(['check-jsonschema', '--schemafile', schema_file, config_file], capture_output=True)
            if result.returncode != 0:
                print(colorama.Fore.RED + '❌ Failed')
                for line in result.stdout.decode('utf-8').split('\n'):
                    if line:
                        print(colorama.Fore.RED + str_indentation + line)
                validation_failed = True
            else:
                print(colorama.Fore.GREEN + '✅ Passed')

    if validation_failed:
        print(colorama.Style.BRIGHT + colorama.Fore.RED + '❗ Validation failed for one or more files.')
        exit(1)
    else:
        print(colorama.Style.BRIGHT + colorama.Fore.GREEN + '✔️ All validations passed successfully.')

if __name__ == "__main__":
    main()
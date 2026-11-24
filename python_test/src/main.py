def main():
    print("Hello from python-test!")
    # python version
    print(f"Python version: {'.'.join(map(str, __import__('sys').version_info[:3]))}")


if __name__ == "__main__":
    main()

# mv ~/(이전에 작업한 폴더)/* ~/(github 에 연결된 폴더)/

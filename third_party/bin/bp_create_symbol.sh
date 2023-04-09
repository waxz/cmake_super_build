#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
dump_syms=$SCRIPT_DIR/dump_syms
minidump_stackwalk=$SCRIPT_DIR/minidump_stackwalk
if [ $# != 1 ] ; then
echo "USAGE: $0 TARGET_NAME DMP_NAME OUTPUT_NAME"
echo " e.g.: $0 test 48596758-1d7a-4318-15edb4af-a9186ad7.dmp error.log"
exit 1;
fi

#获取输入参数
target_file_path=$1
target_file_name="$(basename "${target_file_path}")"
#dmp_file_name=$2
#output_file_name=$3
sym_file_name=$target_file_name'.sym'
echo "target_file_path : " $target_file_path
echo "target_file_name : " $target_file_name
echo "sym_file_name : " $sym_file_name


getSymbol() {
    echo "@getSymbol: start get symbol"
    $dump_syms $target_file_path > $sym_file_name
}

createSymbolDir() {
    echo "@createSymbolDir: start get StackTrace"

    #获取符号文件中的行
    line1=`head -n1 $sym_file_name`

    #从行字符串中获取版本号
    OIFS=$IFS; IFS=" "; set -- $line1; aa=$1;bb=$2;cc=$3;dd=$4; IFS=$OIFS

    version_number=$dd

    #创建特定的目录结构，并将符号文件移进去
    mkdir -p ./symbols/$target_file_name/$version_number
    mv $sym_file_name ./symbols/$target_file_name/$version_number

    #将堆栈跟踪信息重定向到文件中
#    $minidump_stackwalk $dmp_file_name ./symbols > $output_file_name
}

main() {
    getSymbol
    if [ $? == 0 ]
    then
        createSymbolDir
    fi
}

#运行main
main
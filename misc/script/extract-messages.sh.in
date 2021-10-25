#!/bin/sh
echo "Extracting messages to translate from the source files."
xgettext -k"_" -k"N_" -k"Q_" --package-name=Choreonoid --width=100 --no-location --sort-by-file -o po/messages.pot `find . -name '*.cpp'`

for po in po/*.po
do
    echo "Merging new messages to $po."
    msgmerge --width=100 --no-location --update $po po/messages.pot
done

printf "\nFixing vagrant authentication issue.\n\n"
wget -q https://raw.githubusercontent.com/mitchellh/vagrant/master/keys/vagrant.pub -O ~/.ssh/authorized_keys
chmod 700 ~/.ssh/
chmod 600 ~/.ssh/authorized_keys
chown -R vagrant:vagrant ~/.ssh/

printf "\nAdding MRR user.\n\n"
sleep 5s
sudo adduser mrr
sudo adduser mrr sudo
sudo adduser mrr dialout

printf "\nRemove unnecessary packages...\n\n"
sleep 5s
sudo apt-get install nano
sudo apt-get remove -y --purge libreoffice*
sudo apt-get clean
sudo apt-get autoremove -y

printf "\nRemoving vagrant user from login...\n\n"
sleep 1s
sudo sed -i 's/SystemAccount=false/SystemAccount=true/g' /var/lib/AccountsService/users/vagrant

printf "\nDownloading provisioning script...\n\n"
wget -q /home/mrr/Desktop/setup_vm.sh
chmod +x /home/mrr/Desktop/setup_vm.sh

printf "\nDone.\n\n"
